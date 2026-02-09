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

extern _Bool calibration_flag;


char usb_send_buff[128];
plot_point_t source_plot[MAX_DATA_PLOT];

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

void parse_piano(uint8_t *buf) {
	int tone = buf[1] - 1;
	if (tone < 0) tone = 0;

	if (buf[0] == 0xD3) note_piano[tone] = 1;
	else if (buf[0] == 0xD2) note_piano[tone] = 0;
}

/******************************************************************************************* */

static int cli_parse(char *cmd, char *argv[], int max_args) {
  int argc = 0;

  while (*cmd && argc < max_args) {
    /* skip leading spaces */
    while (*cmd == ' ') {
      cmd++;
    }

    if (*cmd == '\0') {
      break;
    }

    argv[argc++] = cmd;

    /* find end of token */
    while (*cmd && *cmd != ' ') {
      cmd++;
    }

    if (*cmd == '\0') {
      break;
    }

    /* terminate token */
    *cmd = '\0';
    cmd++;
  }

  return argc;
}


/******************************************************************************************* */

static void get_pid_all_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " kp:%f\r\n ki:%f\r\n kd:%f\r\n deadband:%f\r\n max out:%f\r\n",
            title, pid->kp, pid->ki, pid->kd, pid->e_deadband, pid->out_max);
}

static void get_pid_kp_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " kp:%f\r\n",
            title, pid->kp);
}

static void get_pid_ki_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " ki:%f\r\n",
            title, pid->ki);
}

static void get_pid_kd_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " kd:%f\r\n",
            title, pid->kd);
}

static void get_pid_deadband_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " deadband:%f\r\n",
            title, pid->e_deadband);
}

static void get_pid_max_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " max out:%f\r\n",
            title, pid->out_max);
}


static void get_foc_pid_all_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " kp:%f\r\n ki:%f\r\n kd:%f\r\n deadband:%f\r\n max out:%f\r\n",
            title, pid->kp, pid->ki, pid->kd, pid->e_deadband, pid->out_max_dynamic);
}

static void get_foc_pid_max_parameter(PID_Controller_t *pid, const char *title) {
  usb_print("%s control param:\r\n"
            " max out:%f\r\n",
            title, pid->out_max_dynamic);
}

/******************************************************************************************* */

static void print_mode(motor_mode_t mode) {
  char write_buffer[64];
  uint16_t len = 0;

  switch (mode) {
  case TORQUE_CONTROL_MODE:
    len = snprintf(write_buffer, sizeof(write_buffer), "Control Mode[0]: Current Control\n");
    break;
  case SPEED_CONTROL_MODE:
    len = snprintf(write_buffer, sizeof(write_buffer), "Control Mode[1]: Speed Control\n");
    break;
  case POSITION_CONTROL_MODE:
    len = snprintf(write_buffer, sizeof(write_buffer), "Control Mode[2]: Position Control\n");
    break;
  case CALIBRATION_MODE:
    len = snprintf(write_buffer, sizeof(write_buffer), "Control Mode[3]: Calibration\n");
    break;
  case AUDIO_MODE:
    len = snprintf(write_buffer, sizeof(write_buffer), "Control Mode[4]: Audio\n");
    break;
  default:
    len = snprintf(write_buffer, sizeof(write_buffer), "Wrong Mode\n");
    break;
  }
  CDC_Transmit_FS((uint8_t*)write_buffer, len);
}

/******************************************************************************************* */

static void print_motor_param(void) {
  usb_print("motor param:\r\n"
            " Rs:%f\r\n" 
            " Ld:%f\r\n" 
            " Lq:%f\r\n" 
            " pole pairs:%d\r\n" 
            " Kv:%f\r\n", 
            hfoc.Rs, hfoc.Ld, hfoc.Lq, hfoc.pole_pairs, hfoc.kv);
}

/******************************************************************************************* */

static void add_plot_point(const char *cmd) {
  int total_points = 0;
  float *point = NULL;
  _Bool is_point_available = 0;
  char point_name[MAX_DATA_PLOT];
  float dummy[MAX_DATA_PLOT] = {0.0f};

  for (int i = 0; source_plot[i].addr != NULL && i < MAX_DATA_PLOT; i++) {
    total_points++;
  }

  // current 
  if (strstr(cmd, "ia")) {
    point = &hfoc.ia;
    snprintf(point_name, sizeof(point_name), "ia");
  }
  else if (strstr(cmd, "ib")) {
    point = &hfoc.ib;
    snprintf(point_name, sizeof(point_name), "ib");
  }
  else if (strstr(cmd, "ic")) {
    point = &hfoc.ic;
    snprintf(point_name, sizeof(point_name), "ic");
  }
  else if (strstr(cmd, "i_alpha")) {
    point = &hfoc.i_alpha;
    snprintf(point_name, sizeof(point_name), "i_alpha");
  }
  else if (strstr(cmd, "i_beta")) {
    point = &hfoc.i_beta;
    snprintf(point_name, sizeof(point_name), "i_beta");
  }
  else if (strstr(cmd, "id")) {
    point = &hfoc.id;
    snprintf(point_name, sizeof(point_name), "id");
  }
  else if (strstr(cmd, "iq")) {
    point = &hfoc.iq;
    snprintf(point_name, sizeof(point_name), "iq");
  }
  // voltage
  else if (strstr(cmd, "va")) {
    point = &hfoc.va;
    snprintf(point_name, sizeof(point_name), "va");
  }
  else if (strstr(cmd, "vb")) {
    point = &hfoc.vb;
    snprintf(point_name, sizeof(point_name), "vb");
  }
  else if (strstr(cmd, "vc")) {
    point = &hfoc.vc;
    snprintf(point_name, sizeof(point_name), "vc");
  }
  else if (strstr(cmd, "v_alpha")) {
    point = &hfoc.v_alpha;
    snprintf(point_name, sizeof(point_name), "v_alpha");
  }
  else if (strstr(cmd, "v_beta")) {
    point = &hfoc.v_beta;
    snprintf(point_name, sizeof(point_name), "v_beta");
  }
  else if (strstr(cmd, "vd")) {
    point = &hfoc.vd;
    snprintf(point_name, sizeof(point_name), "vd");
  }
  else if (strstr(cmd, "vq")) {
    point = &hfoc.vq;
    snprintf(point_name, sizeof(point_name), "vq");
  }
  // v bus
  else if (strstr(cmd, "v_bus")) {
    point = &hfoc.v_bus;
    snprintf(point_name, sizeof(point_name), "v_bus");
  }
  
  else if (strstr(cmd, "rpm")) {
    point = &hfoc.actual_rpm;
    snprintf(point_name, sizeof(point_name), "rpm");
  }
  else if (strstr(cmd, "e_rad")) {
    point = &hfoc.e_rad;
    snprintf(point_name, sizeof(point_name), "e_rad");
  }
  else if (strstr(cmd, "m_deg")) {
    point = &hfoc.actual_angle;
    snprintf(point_name, sizeof(point_name), "m_deg");
  }

  if (point == NULL) return;
  for (int i = 0; source_plot[i].addr != NULL && i < MAX_DATA_PLOT; i++) {
    if (source_plot[i].addr == point){
      is_point_available = 1;
    }
  }
  if (!is_point_available) {
    source_plot[total_points].addr = point;
    memcpy(source_plot[total_points].name, point_name, MAX_NAME_POINT);
    send_data_float(dummy, total_points+1); osDelay(1);
    change_legend(total_points, point_name); osDelay(2);
    // erase_graph(); osDelay(1);
  }
}

static void remove_plot_point(const char *cmd) {
  int total_points = 0;
  float *point = NULL;
  _Bool is_point_available = 0;
  uint8_t start_idx;
  float dummy[MAX_DATA_PLOT] = {0.0f};

  // current 
  if (strstr(cmd, "ia")) {
    point = &hfoc.ia;
  }
  else if (strstr(cmd, "ib")) {
    point = &hfoc.ib;
  }
  else if (strstr(cmd, "ic")) {
    point = &hfoc.ic;
  }
  else if (strstr(cmd, "i_alpha")) {
    point = &hfoc.i_alpha;
  }
  else if (strstr(cmd, "i_beta")) {
    point = &hfoc.i_beta;
  }
  else if (strstr(cmd, "id")) {
    point = &hfoc.id;
  }
  else if (strstr(cmd, "iq")) {
    point = &hfoc.iq;
  }
  // voltage
  else if (strstr(cmd, "va")) {
    point = &hfoc.va;
  }
  else if (strstr(cmd, "vb")) {
    point = &hfoc.vb;
  }
  else if (strstr(cmd, "vc")) {
    point = &hfoc.vc;
  }
  else if (strstr(cmd, "v_alpha")) {
    point = &hfoc.v_alpha;
  }
  else if (strstr(cmd, "v_beta")) {
    point = &hfoc.v_beta;
  }
  else if (strstr(cmd, "vd")) {
    point = &hfoc.vd;
  }
  else if (strstr(cmd, "vq")) {
    point = &hfoc.vq;
  }
  // v bus
  else if (strstr(cmd, "v_bus")) {
    point = &hfoc.v_bus;
  }

  else if (strstr(cmd, "rpm")) {
    point = &hfoc.actual_rpm;
  }
  else if (strstr(cmd, "e_rad")) {
    point = &hfoc.e_rad;
  }
  else if (strstr(cmd, "m_deg")) {
    point = &hfoc.actual_angle;
  }

  if (point == NULL) return;
  for (int i = 0; source_plot[i].addr != NULL && i < MAX_DATA_PLOT; i++) {
    if (source_plot[i].addr == point){
      start_idx = i;
      is_point_available = 1;
    }
  }
  if (is_point_available) {
    for (int i = start_idx; i < MAX_DATA_PLOT-1; i++) {
      source_plot[i].addr = source_plot[i+1].addr;
      memcpy(source_plot[i].name, source_plot[i+1].name, MAX_NAME_POINT);
    }
    source_plot[MAX_DATA_PLOT-1].addr = NULL;
    memset(source_plot[MAX_DATA_PLOT-1].name, '\0', MAX_NAME_POINT);
    
    for (int i = 0; source_plot[i].addr != NULL && i < MAX_DATA_PLOT; i++) {
      total_points++;
    }

    erase_graph(); osDelay(1);
    send_data_float(dummy, total_points); osDelay(1);
    for (int i = 0; i < total_points; i++) {
      change_legend(i, source_plot[i].name); osDelay(2);
    }
  }
}

void parse_command(char *cmd) {
  char *argv[CLI_MAX_ARGS];
  int argc = cli_parse(cmd, argv, CLI_MAX_ARGS);
  _Bool need_to_save = 0;

  if (strstr(argv[0], "plot")) {
    if (strstr(argv[1], "add")) {
      if (argc == 3) {
        add_plot_point(argv[2]);
      }
    }
    else if (strstr(argv[1], "rm")) {
      if (argc == 3) {
        remove_plot_point(argv[2]);
      }
    }
  }
  else if (strstr(argv[0], "pid")) {
    PID_Controller_t *pid = NULL;
    _Bool foc = 0;

    if (strstr(argv[1], "id")) {
      pid = &hfoc.id_ctrl;
      foc = 1;
    }
    else if (strstr(argv[1], "iq")) {
      pid = &hfoc.iq_ctrl;
      foc = 1;
    }
    else if (strstr(argv[1], "speed")) {
      pid = &hfoc.speed_ctrl;
    }
    else if (strstr(argv[1], "position")) {
      pid = &hfoc.pos_ctrl;
    }

    if (pid == NULL) return;

    if (argc == 2) {
      if (foc) {
        get_foc_pid_all_parameter(pid, argv[1]);
      }
      else {
        get_pid_all_parameter(pid, argv[1]);
      }
    }
    else if (argc >= 3) {
      float val = 0.0f;

      if (strstr(argv[2], "bw") && foc) {
		    if (argc == 4) {
          val = atof(argv[3]);
          hfoc.I_ctrl_bandwidth = val;
          m_config.I_ctrl_bandwidth = val;
          flash_auto_tuning_torque_control(&m_config);
          hfoc.id_ctrl.kp = m_config.id_kp;
          hfoc.id_ctrl.ki = m_config.id_ki;
          hfoc.iq_ctrl.kp = m_config.iq_kp;
          hfoc.iq_ctrl.ki = m_config.iq_ki;
          usb_print(" new bw:%.2f\r\n", hfoc.I_ctrl_bandwidth);
        }
        else {
          usb_print(" bw:%.2f\r\n", hfoc.I_ctrl_bandwidth);
        }
      }
      else if (strstr(argv[2], "kp")) {
		    if (argc == 4) {
          val = atof(argv[3]);
          pid->kp = val;
          need_to_save = 1;
        }
        else {
          get_pid_kp_parameter(pid, argv[1]);
        }
      }
      else if (strstr(argv[2], "ki")) {
		    if (argc == 4) {
          val = atof(argv[3]);
          pid->ki = val;
          need_to_save = 1;
        }
        else {
          get_pid_ki_parameter(pid, argv[1]);
        }
      }
      else if (strstr(argv[2], "kd")) {
		    if (argc == 4) {
          val = atof(argv[3]);
          pid->kd = val;
          need_to_save = 1;
        }
        else {
          get_pid_kd_parameter(pid, argv[1]);
        }
      }
      else if (strstr(argv[2], "deadband")) {
		    if (argc == 4) {
          val = atof(argv[3]);
          pid->e_deadband = val;
          need_to_save = 1;
        }
        else {
          get_pid_deadband_parameter(pid, argv[1]);
        }
      }
      else if (strstr(argv[2], "max")) {
		    if (argc == 4) {
          val = atof(argv[3]);
          if (foc) {
            pid->out_max_dynamic = val;
          }
          else {
            pid->out_max = val;
          }
          need_to_save = 1;
        }
        else {
          if (foc) {
            get_foc_pid_max_parameter(pid, argv[1]);
          }
          else {
            get_pid_max_parameter(pid, argv[1]);
          }
        }
      }
    }
  }
  else if (strstr(argv[0], "calib")) {
    usb_print("start callibration\r\n");
    hfoc.control_mode = CALIBRATION_MODE;
    calibration_flag = 1;
  }
	else if (strstr(argv[0], "mode") != NULL) {
    if (argc == 2) {
      motor_mode_t mode = TORQUE_CONTROL_MODE;
      mode = (motor_mode_t)atoi(argv[1]);
      print_mode(mode);

      if (mode <= AUDIO_MODE) {
        hfoc.control_mode = mode;
        sp_input = 0;
      }
    }
	}
	else if (strstr(argv[0], "sp")) {
    if (argc == 2) {
      sp_input = atof(argv[1]);
    }
	}
  else if (strstr(argv[0], "ratio") != NULL) {
    if (argc == 2) {
      float ratio = atof(argv[1]);
      if (ratio <= 0.0f) return;
      hfoc.gear_ratio = ratio;

      need_to_save = 1;
    }
  }
  else if (strstr(argv[0], "save") != NULL) {
    flash_save_config(&m_config);
    usb_print(" success save configuration\r\n");
  }
  else if (strstr(argv[0], "set_default") != NULL) {
    flash_default_config(&m_config);
    usb_print(" success reset configuration\r\n");
  }
  else if (strstr(argv[0], "motor_param") != NULL) {
    print_motor_param();
  }

  if (need_to_save) {
    copy_from_local(&m_config, &hfoc);
    usb_print(" OK\r\n");
  }
}
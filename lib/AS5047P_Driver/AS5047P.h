/*
 * AS5047P.h
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#ifndef AS5047P_H_
#define AS5047P_H_

#include <stdint.h>
#include "AS5047P_Config.h"

typedef enum {
    SENSOR_DIR_NORMAL,
    SENSOR_DIR_REVERSE
}sensor_dir_t;

typedef struct {
    uint8_t spi_rx_buffer[2];
    
    uint32_t raw_pos;
    float angle_filtered;
    float prev_angle_filtered;
    float prev_raw_angle;
    uint8_t spike_counter;
    
    float prev_angle;
    float filtered_rpm;
    float prev_rpm;
    float angle_accumulator;
    uint32_t time_accumulator;

    sensor_dir_t dir;
    float count_to_deg_scale;
    
	float output_prev_angle;
	float output_angle_ovf;
    float output_angle_filtered;

    float angle_alpha_filter;
    float rpm_alpha_filter;

    _Bool spi_transfer_flag;
    _Bool spi_transfer_done_flag;

    int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t);
    void (*spi_cs)(_Bool);
}AS5047P_t;

void AS5047P_spi_config(AS5047P_t *encd, int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t), void (*spi_cs)(_Bool));
void AS5047P_init(AS5047P_t *encd, sensor_dir_t dir, float scale);
void AS5047P_set_angle_filter_fc(AS5047P_t *encd, float fc, float Ts);
void AS5047P_set_rpm_filter_fc(AS5047P_t *encd, float fc, float Ts);
int AS5047P_start(AS5047P_t *encd);
void AS5047P_calc_degree(AS5047P_t *encd);
void AS5047P_update(AS5047P_t *encd);
void AS5047P_set_spi_transfer_done(AS5047P_t *encd);
float AS5047P_get_degree(AS5047P_t *encd);
float AS5047P_get_rpm(AS5047P_t *encd, float Ts);
float AS5047P_get_actual_degree(AS5047P_t *encd);

#endif /* AS5047P_H_ */

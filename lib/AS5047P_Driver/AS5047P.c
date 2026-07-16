/*
 * AS5047P.C
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#include "AS5047P.h"
#include <string.h>
#include <math.h>

#ifndef TWO_PI
#define TWO_PI 6.2831853f
#endif

#define AS5047P_REG_ANGLE  0x3FFF
#define AS5047P_WRITE_CMD  0x4000

static uint8_t calc_even_parity(uint16_t value) {
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}

void AS5047P_spi_config(AS5047P_t *encd, int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t), void (*spi_cs)(_Bool)) {
    encd->spi_transfer  = spi_transfer;
    encd->spi_cs  = spi_cs;
    encd->spi_cs(1);
}

void AS5047P_init(AS5047P_t *encd, sensor_dir_t dir, float scale) {
    encd->dir = dir;
    encd->count_to_deg_scale = scale;
}

void AS5047P_set_angle_filter_fc(AS5047P_t *encd, float fc, float Ts) {
    float tau = 1.0f / (TWO_PI * fc);
    encd->angle_alpha_filter = Ts / (tau + Ts);
    if (encd->angle_alpha_filter > 1.0f) encd->angle_alpha_filter = 1.0f;
}

void AS5047P_set_rpm_filter_fc(AS5047P_t *encd, float fc, float Ts) {
    float tau = 1.0f / (TWO_PI * fc);
    encd->rpm_alpha_filter = Ts / (tau + Ts);
    if (encd->rpm_alpha_filter > 1.0f) encd->rpm_alpha_filter = 1.0f;
}

int AS5047P_start(AS5047P_t *encd) {
    if (encd->spi_transfer_flag) return 0;
    uint16_t cmd = AS5047P_WRITE_CMD | AS5047P_REG_ANGLE;
    cmd |= (calc_even_parity(cmd) << 15);  // bit 15 parity

	encd->spi_cs(0);
	if (encd->spi_transfer((uint8_t*)&cmd, encd->spi_rx_buffer, 2) != 0) {
        return -1;
    }

    encd->spi_transfer_flag = 1;
	return 0;
}

void AS5047P_calc_degree(AS5047P_t *encd) {
    if (!encd->spi_transfer_flag) return;
    encd->spi_transfer_flag = 0;
    encd->spi_cs(1);

    const uint16_t raw_data = ((uint16_t)encd->spi_rx_buffer[0] << 8) | encd->spi_rx_buffer[1];

    // Parity check (bit 15 = parity)
    const uint16_t data_15bit = raw_data & 0x7FFF;
    const uint8_t expected_parity = calc_even_parity(data_15bit);
    const uint8_t received_parity = (raw_data >> 15) & 0x1;
    if (expected_parity != received_parity) {
        return;
    }

    // Error flag check (bit 14)
    if ((raw_data >> 14) & 0x1) {
        return;
    }

    uint16_t pos = raw_data & 0x3FFF;
    encd->raw_pos = (encd->dir == SENSOR_DIR_NORMAL)? pos : (0x3FFF - pos);
    const float angle_raw = (float)encd->raw_pos * encd->count_to_deg_scale;

    // encd->angle_filtered = encd->angle_filtered * (1.0f - encd->angle_alpha_filter) + angle_raw * encd->angle_alpha_filter;

    // float angle_diff = angle_raw - encd->prev_raw_angle;
    // angle_diff -= 360.0f * floorf((angle_diff + 180.0f) / 360.0f);

    // if (fabsf(angle_diff) > MAX_ANGLE_JUMP_DEG) {
    //     if (++encd->spike_counter < SPIKE_REJECT_COUNT) {
    //         return;
    //     }
    //     encd->spike_counter = 0;
    // } else {
    //     encd->spike_counter = 0;
    // }

    // encd->prev_raw_angle = angle_raw;

    // Filter IIR dengan wrap-around
    float filtered_diff = angle_raw - encd->angle_filtered;
    filtered_diff -= 360.0f * floorf((filtered_diff + 180.0f) / 360.0f);
    encd->angle_filtered += encd->angle_alpha_filter * filtered_diff;

    if (encd->angle_filtered >= 360.0f)
        encd->angle_filtered -= 360.0f;
    else if (encd->angle_filtered < 0.0f)
        encd->angle_filtered += 360.0f;
}

void AS5047P_update(AS5047P_t *encd) {
    AS5047P_start(encd);
    if (encd->spi_transfer_done_flag) {
        encd->spi_transfer_done_flag = 0;
        AS5047P_calc_degree(encd);
    }
}

void AS5047P_set_spi_transfer_done(AS5047P_t *encd) {
    encd->spi_transfer_done_flag = 1;
}

float AS5047P_get_degree(AS5047P_t *encd) {
    return encd->angle_filtered;
}

float AS5047P_get_rpm(AS5047P_t *encd, float Ts) {
    // Handle angle wrap-around (optimized)
    float angle_diff = encd->angle_filtered - encd->prev_angle;
    angle_diff -= 360.0f * floorf((angle_diff + 180.0f) * (1.0f/360.0f));
    encd->prev_angle = encd->angle_filtered;

    // Calculate RPM
    float rpm_instant = (angle_diff * 60.0f) / (Ts * DEGREES_PER_REV);

    // IIR Filter with dynamic weighting
    float filtered = encd->filtered_rpm * (1.0f - encd->rpm_alpha_filter) + rpm_instant * encd->rpm_alpha_filter;

    // Very low RPM clamping (0.1 RPM resolution)
    // if (fabsf(filtered) < 0.1f) {
    //     filtered = 0.0f;
    // }

    // Update state
    encd->prev_rpm = rpm_instant;
    encd->filtered_rpm = filtered;

    return encd->filtered_rpm;
}

float AS5047P_get_actual_degree(AS5047P_t *encd) {
    const float m_current_angle = encd->angle_filtered;
	float angle_dif = (m_current_angle - encd->output_prev_angle);

	if (angle_dif< -180) {
		encd->output_angle_ovf++;
	}
	else if (angle_dif> 180) {
		encd->output_angle_ovf--;
	}
	float out_deg = (m_current_angle + encd->output_angle_ovf * 360.0f + ACTUAL_ANGLE_OFFSET);
    encd->output_angle_filtered = (1.0f - ACTUAL_ANGLE_FILTER_ALPHA) * encd->output_angle_filtered + ACTUAL_ANGLE_FILTER_ALPHA * out_deg;
	encd->output_prev_angle = m_current_angle;

	// return out_deg;
    return encd->output_angle_filtered;
}


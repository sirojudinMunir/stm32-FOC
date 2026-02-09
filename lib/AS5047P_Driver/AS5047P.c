/*
 * AS5047P.C
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#include "AS5047P.h"
#include <string.h>
#include <math.h>

#define AS5047P_REG_ANGLE  0x3FFF
#define AS5047P_WRITE_CMD  0x4000

_Bool encd_get_val_flag = 0;

uint8_t calc_even_parity(uint16_t value) {
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}


int AS5047P_config(AS5047P_t *encd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    if (encd == NULL || hspi == NULL || cs_port == NULL || cs_pin == 0) {
        return 0;
    }

    encd->AS5047P_spi = hspi;
    encd->AS5047P_cs_port = cs_port;
    encd->AS5047P_cs_pin = cs_pin;
    
    AS5047P_cs_set(encd);
    
    return 1;
}

int AS5047P_readCommand(AS5047P_t *encd, uint8_t *cmd, uint8_t *receive_buffer) {
	AS5047P_cs_reset(encd);

	if (HAL_SPI_TransmitReceive(encd->AS5047P_spi, cmd, receive_buffer, 2, 1000) != HAL_OK) {
	    AS5047P_cs_set(encd);
        return 0;
    }

    AS5047P_cs_set(encd);
	return 1;
}

int AS5047P_start(AS5047P_t *encd) {
    uint16_t cmd = AS5047P_WRITE_CMD | AS5047P_REG_ANGLE;
    cmd |= (calc_even_parity(cmd) << 15);  // bit 15 parity

	AS5047P_cs_reset(encd);
	if (HAL_SPI_TransmitReceive_DMA(encd->AS5047P_spi, (uint8_t*)&cmd, encd->spi_rx_buffer, 2) != HAL_OK) {
        return 0;
    }

	return 1;
}


float AS5047P_get_degree(AS5047P_t *encd) {
    AS5047P_cs_set(encd);

    const uint16_t raw_data = ((uint16_t)encd->spi_rx_buffer[0] << 8) | encd->spi_rx_buffer[1];

    // Parity check (bit 15 = parity)
    const uint16_t data_15bit = raw_data & 0x7FFF;
    const uint8_t expected_parity = calc_even_parity(data_15bit);
    const uint8_t received_parity = (raw_data >> 15) & 0x1;
    if (expected_parity != received_parity) {
        return encd->angle_filtered;
    }

    // Error flag check (bit 14)
    if ((raw_data >> 14) & 0x1) {
        return encd->angle_filtered;
    }

#ifdef HIGH_RES
    const float angle_raw = (float)(raw_data & 0x3FFF) * ANGLE_SCALE_FACTOR;
#else
    const float angle_raw = (float)((raw_data >> 2) & 0x0FFF) * ANGLE_SCALE_FACTOR;
#endif

    float angle_diff = angle_raw - encd->prev_raw_angle;
    angle_diff -= 360.0f * floorf((angle_diff + 180.0f) / 360.0f);

    if (fabsf(angle_diff) > MAX_ANGLE_JUMP_DEG) {
        if (++encd->spike_counter < SPIKE_REJECT_COUNT) {
            return encd->angle_filtered;
        }
        encd->spike_counter = 0;
    } else {
        encd->spike_counter = 0;
    }

    encd->prev_raw_angle = angle_raw;

    // Filter IIR dengan wrap-around
    float filtered_diff = angle_raw - encd->angle_filtered;
    filtered_diff -= 360.0f * floorf((filtered_diff + 180.0f) / 360.0f);
    encd->angle_filtered += ANGLE_FILTER_ALPHA * filtered_diff;

    if (encd->angle_filtered >= 360.0f)
        encd->angle_filtered -= 360.0f;
    else if (encd->angle_filtered < 0.0f)
        encd->angle_filtered += 360.0f;

    return encd->angle_filtered;
}


float AS5047P_get_rpm(AS5047P_t *encd, float Ts) {
    // Handle angle wrap-around (optimized)
    float angle_diff = encd->angle_filtered - encd->prev_angle;
    angle_diff -= 360.0f * floorf((angle_diff + 180.0f) * (1.0f/360.0f));
    encd->prev_angle = encd->angle_filtered;

    // Calculate RPM
    float rpm_instant = (angle_diff * 60.0f) / (Ts * DEGREES_PER_REV);

    // Two-stage spike rejection
    float rpm_delta = rpm_instant - encd->prev_rpm;
    float abs_delta = fabsf(rpm_delta);

    if (abs_delta > MAX_RPM_JUMP) {
        // Gradual rejection with 50% of the delta, capped at MAX_RPM_JUMP
        float limited_delta = copysignf(fminf(abs_delta * 0.5f, MAX_RPM_JUMP), rpm_delta);
        rpm_instant = encd->prev_rpm + limited_delta;
    }

    // IIR Filter with dynamic weighting
    float filtered = encd->filtered_rpm * (1.0f - RPM_FILTER_ALPHA) + rpm_instant * RPM_FILTER_ALPHA;

    // Very low RPM clamping (0.1 RPM resolution)
    if (fabsf(filtered) < 0.1f) {
        filtered = 0.0f;
    }

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


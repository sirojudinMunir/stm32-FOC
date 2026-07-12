/*
 * CAN.h
 *
 *  Created on: October 14, 2025
 *      Author: munir
 */

#ifndef CAN_H_
#define CAN_H_

#include "stm32f4xx_hal.h"

#define RECEIVER_ID 0x02
#define TRANSMITTER_ID 0x01

/* Extern variable */
extern float test_angle_sp;

void CAN_FilterConfig(CAN_HandleTypeDef *hcan);
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len);
void CAN_init(CAN_HandleTypeDef *hcan);

#endif

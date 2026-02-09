/*
 * AS5047P.h
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#ifndef AS5047P_DRIVER_INC_AS5047P_H_
#define AS5047P_DRIVER_INC_AS5047P_H_

#include "AS5047P_Config.h"

#ifdef HIGH_RES
#define ANGLE_SCALE_FACTOR    0.021973997F  // Pre-calculate scale factor (360.0f / 16383.0f)
#else
#define ANGLE_SCALE_FACTOR    (360.0f / 4095.0f) 
#endif


typedef struct {
    SPI_HandleTypeDef *AS5047P_spi;
    GPIO_TypeDef *AS5047P_cs_port;  
    uint16_t AS5047P_cs_pin;        
    uint8_t spi_rx_buffer[2];
    
    float angle_filtered;
    float prev_angle_filtered;
    float prev_raw_angle;
    uint8_t spike_counter;
    
    float prev_angle;
    float filtered_rpm;
    float prev_rpm;
    float angle_accumulator;
    uint32_t time_accumulator;
    
	float output_prev_angle;
	float output_angle_ovf;
    float output_angle_filtered;
}AS5047P_t;
   
#define AS5047P_cs_set(encd) ((encd)->AS5047P_cs_port->BSRR = (encd)->AS5047P_cs_pin)
#define AS5047P_cs_reset(encd) ((encd)->AS5047P_cs_port->BSRR = (encd)->AS5047P_cs_pin<<16)

#define AS5047P_get_val_flag() (encd_get_val_flag == 1)
#define AS5047P_set_val_flag() (encd_get_val_flag = 1)
#define AS5047P_reset_val_flag() (encd_get_val_flag = 0)


extern _Bool encd_get_val_flag;

int AS5047P_config(AS5047P_t *encd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
int AS5047P_start(AS5047P_t *encd);
float AS5047P_get_degree(AS5047P_t *encd);
float AS5047P_get_rpm(AS5047P_t *encd, float Ts);
float AS5047P_get_actual_degree(AS5047P_t *encd);

int AS5047P_readCommand(AS5047P_t *encd, uint8_t *cmd, uint8_t *receive_buffer);

#endif /* AS5047P_DRIVER_INC_AS5047P_H_ */

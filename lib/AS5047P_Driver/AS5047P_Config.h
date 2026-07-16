/*
 * AS5047P_Config.h
 *
 *  Created on: Jun 27, 2025
 *      Author: munir
 */

#ifndef AS5047P_DRIVER_INC_AS5047P_CONFIG_H_
#define AS5047P_DRIVER_INC_AS5047P_CONFIG_H_

#define HIGH_RES

#define ANGLE_FILTER_ALPHA    0.71539f  //0.71539f //   // Faktor filter
#define MAX_ANGLE_JUMP_DEG    50.0f   // Batas maksimal lonjakan sudut (derajat)
#define SPIKE_REJECT_COUNT    10       // Jumlah sampel untuk konfirmasi spike

// Configurations (tune these based on your system)
#define MIN_DT_US             500UL      // Minimum 1ms interval for valid RPM (avoid division by tiny numbers)
#define MAX_RPM_JUMP          100.0f      // Conservative RPM jump threshold
#define RPM_FILTER_ALPHA      0.5f //0.71539f      // Base filter coefficient (balanced response)
#define DEGREES_PER_REV       360.0f     // For 1:1 gear ratio
#define MICROS_TO_MINUTES     6e7f       // Conversion factor (μs to minutes)

#define GEAR_RATIO 1.0f //0.0526315789f
#define ACTUAL_ANGLE_OFFSET 60.0f
// #define ACTUAL_ANGLE_OFFSET (-75.0f)
#define ACTUAL_ANGLE_FILTER_ALPHA 0.71539f

#endif /* AS5047P_DRIVER_AS5047P_CONFIG_H_ */

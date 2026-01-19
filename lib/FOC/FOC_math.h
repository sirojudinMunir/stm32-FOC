/*
 * FOC_math.h
 *
 *  Created on: Jul 12, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_MATH_H_
#define FOC_INC_FOC_MATH_H_


#include <stdint.h>
#include <complex.h>

#define LUT_SIZE (1024*4)
#define LUT_STEP (TWO_PI / (float)LUT_SIZE)

#define TWO_BY_SQRT3 1.15470053838f
#define ONE_BY_SQRT3 0.57735026919f
#define SQRT3_BY_TWO 0.86602540378f
#define SQRT3 1.73205080757f
#define TWO_PI 6.2831853f
#define PI 3.1415926f

#define CONSTRAIN(val, min, max) \
    ((val) <= (min) ? (min) : ((val) >= (max) ? (max) : (val)))

#define RAD_TO_DEG(rad) ((rad) * 57.29577951308232f)  // 180/π

#define DEG_TO_RAD(deg) ((deg) * 0.017453292519943f)  // π/180

typedef struct {
    float real;
    float imag;
} complex_t;

extern float sin_lut[LUT_SIZE];
extern float cos_lut[LUT_SIZE];

void init_trig_lut(void);
void norm_angle_rad(float *theta);
float fast_sin(float theta);
float fast_cos(float theta);
float fast_atan2(float y, float x);
complex_t complex_multiply(complex_t a, complex_t b);
complex_t complex_add(complex_t a, complex_t b);
complex_t complex_subtract(complex_t a, complex_t b);
void pre_calc_sin_cos(float theta, float *sin_theta, float *cos_theta);

void clarke_transform(float ia, float ib, float *i_alpha, float *i_beta);
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float *id, float *iq);
void clarke_park_transform(float ia, float ib, float sin_theta, float cos_theta, float *id, float *iq);
void inverse_park_transform(float vd, float vq, float sin_theta, float cos_theta, float *valpha, float *vbeta);
void inverse_clarke_transform(float valpha, float vbeta, float *va, float *vb, float *vc);
void svpwm(float valpha, float vbeta, float vbus, uint32_t pwm_period,
			uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w);

void vabc_to_vdq(float va, float vb, float vc, float sin_theta, float cos_theta, float *vd, float *vq);

#endif /* FOC_INC_FOC_MATH_H_ */

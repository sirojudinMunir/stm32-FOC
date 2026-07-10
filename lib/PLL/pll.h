#ifndef __PLL_H__
#define __PLL_H__

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float k_h;           
    float integral;
    float omega_est;    
    float theta_est;     
	float last_theta_est;
    float omega_max;
} pll_t;

void pll_init(pll_t* pll, float kp, float ki, float k_h, float omega_max);
float pll_update(pll_t* pll, float error, float Ts);
void pll_reset(pll_t *pll);

#endif
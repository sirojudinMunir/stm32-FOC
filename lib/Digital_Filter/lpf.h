#ifndef __LPF_H__
#define __LPF_H__

#include <stdint.h>

typedef struct {
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;      // Denominator coefficients
    float x1, x2;      // Input delays
    float y1, y2;      // Output delays
} SecondOrderLPF;

void second_order_lpf_init(SecondOrderLPF* lpf, float cutoff_freq, float sampling_freq);
float second_order_lpf_update(SecondOrderLPF* lpf, float input);

#endif
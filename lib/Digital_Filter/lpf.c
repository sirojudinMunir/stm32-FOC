#include "lpf.h"
#include "FOC_math.h"
#include "math.h"

void second_order_lpf_init(SecondOrderLPF* lpf, float cutoff_freq, float sampling_freq) {
    float w0 = 2.0f * PI * cutoff_freq / sampling_freq;
    float Q = 0.7071f; // Butterworth Q factor 1/sqrt(2)
    float alpha = sinf(w0) / (2.0f * Q);
    
    float cos_w0 = cosf(w0);
    float a0 = 1.0f + alpha;
    
    // Butterworth coefficients
    lpf->b0 = (1.0f - cos_w0) / (2.0f * a0);
    lpf->b1 = (1.0f - cos_w0) / a0;
    lpf->b2 = lpf->b0;
    lpf->a1 = -2.0f * cos_w0 / a0;
    lpf->a2 = (1.0f - alpha) / a0;
    
    // Reset states
    lpf->x1 = lpf->x2 = 0.0f;
    lpf->y1 = lpf->y2 = 0.0f;
}

float second_order_lpf_update(SecondOrderLPF* lpf, float input) {
    float output = lpf->b0 * input + lpf->b1 * lpf->x1 + lpf->b2 * lpf->x2
                   - lpf->a1 * lpf->y1 - lpf->a2 * lpf->y2;
    
    // Update delays
    lpf->x2 = lpf->x1;
    lpf->x1 = input;
    lpf->y2 = lpf->y1;
    lpf->y1 = output;
    
    return output;
}

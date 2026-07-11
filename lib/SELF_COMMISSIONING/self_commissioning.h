#ifndef SELF_COMMISSIONING_H
#define SELF_COMMISSIONING_H

#include <stdint.h>
#include "math.h"
#include "FOC_math.h"
#include "FOC_utils.h"

#define MAX_DATA_ACQ_BUFFER  (256)

typedef enum {
    SC_SEQUENCE_START_MEASURE_RS,
    SC_SEQUENCE_START_MEASURE_LD,
    SC_SEQUENCE_START_MEASURE_LQ,
}sc_sequence_t;

typedef struct {
    sc_sequence_t seq;
    float signal_amplitude;
    float signal_omega;
    float signal_offset;
    uint32_t signal_delay_t;
    uint32_t signal_t;
    uint32_t signal_start_t;
    _Bool signal_flag;
    _Bool measure_done_flag;
    foc_t *p_foc;
    float v_buffer[MAX_DATA_ACQ_BUFFER];
    float i_buffer[MAX_DATA_ACQ_BUFFER];
    float Rs;
    float Ld;
    float Lq;
}self_commissioning_t;

void sc_init(self_commissioning_t *sc, foc_t *hfoc);
int sc_start_measure_motor_resistance(self_commissioning_t *sc);
int sc_start_measure_motor_Ld(self_commissioning_t *sc);
int sc_start_measure_motor_Lq(self_commissioning_t *sc);
void sc_update(self_commissioning_t *sc, float Ts);
_Bool sc_is_measure_done(self_commissioning_t *sc);
sc_sequence_t sc_get_seq(self_commissioning_t *sc);
float sc_get_Rs(self_commissioning_t *sc);
float sc_get_Ld(self_commissioning_t *sc);
float sc_get_Lq(self_commissioning_t *sc);

#endif // SELF_COMMISSIONING_H

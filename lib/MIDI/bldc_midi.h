#ifndef __BLDC_MIDI_H__
#define __BLDC_MIDI_H__

#include <stdint.h>
#include "FOC_utils.h"

#define MAX_NOTES 7
#define AUDIO_SAMPLE_RATE 10000
#define MAX_TONE_PIANO 62

extern float v_tone;

void audio_loop(foc_t *hfoc);

#endif
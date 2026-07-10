
#include "bldc_midi.h"
#include "math.h"
#include "stm32f4xx_hal.h"

// Pre-calculated frequency table for all MIDI notes (0-127)
static const float MIDI_NOTE_FREQ[128] = {
    8.175798f,    8.661957f,    9.177024f,    9.722718f,    10.300861f,   10.913382f,   11.562325f,   12.249857f,
    12.978271f,   13.750000f,   14.567617f,   15.433853f,   16.351597f,   17.323914f,   18.354048f,   19.445436f,
    20.601722f,   21.826764f,   23.124651f,   24.499714f,   25.956543f,   27.500000f,   29.135235f,   30.867706f,
    32.703193f,   34.647827f,   36.708096f,   38.890873f,   41.203445f,   43.653529f,   46.249303f,   48.999429f,
    51.913086f,   55.000000f,   58.270470f,   61.735413f,   65.406387f,   69.295654f,   73.416192f,   77.781746f,
    82.406889f,   87.307058f,   92.498606f,   97.998859f,   103.826172f,  110.000000f,  116.540940f,  123.470825f,
    130.812775f,  138.591309f,  146.832384f,  155.563492f,  164.813778f,  174.614116f,  184.997211f,  195.997718f,
    207.652344f,  220.000000f,  233.081881f,  246.941650f,  261.625549f,  277.182617f,  293.664768f,  311.126984f,
    329.627556f,  349.228231f,  369.994423f,  391.995436f,  415.304688f,  440.000000f,  466.163757f,  493.883301f,
    523.251099f,  554.365234f,  587.329536f,  622.253967f,  659.255112f,  698.456463f,  739.988846f,  783.990875f,
    830.609375f,  880.000000f,  932.327515f,  987.766602f,  1046.502197f, 1108.730469f, 1174.659073f, 1244.507935f,
    1318.510225f, 1396.912926f, 1479.977692f, 1567.981750f, 1661.218750f, 1760.000000f, 1864.655029f, 1975.533203f,
    2093.004395f, 2217.460938f, 2349.318146f, 2489.015869f, 2637.020450f, 2793.825852f, 2959.955385f, 3135.963500f,
    3322.437500f, 3520.000000f, 3729.310059f, 3951.066406f, 4186.008789f, 4434.921875f, 4698.636292f, 4978.031738f,
    5274.040901f, 5587.651704f, 5919.910769f, 6271.927000f, 6644.875000f, 7040.000000f, 7458.620117f, 7902.132812f,
    8372.017578f, 8869.843750f, 9397.272461f, 9956.063477f, 10548.081787f,11175.303406f,11839.821533f,12543.854004f
};

int note_piano[MAX_TONE_PIANO];
_Bool tone_state[MAX_TONE_PIANO];
float tone_amp[MAX_TONE_PIANO];
uint32_t decay_tick[MAX_TONE_PIANO];

float v_tone;

static float note_freq(int note_number) {
    // Clamp the note number to valid MIDI range (0-127)
    if (note_number < 0) return MIDI_NOTE_FREQ[0];
    if (note_number > 127) return MIDI_NOTE_FREQ[127];
    return MIDI_NOTE_FREQ[note_number];
}

void audio_loop(foc_t *hfoc) {
    static uint32_t sample_count = 0;
    const float dt = 1.0f / AUDIO_SAMPLE_RATE;
    const uint32_t decay_time = 1500;

    float t = sample_count * dt;

    // Find all active notes (up to MAX_NOTES)
    float active_freqs[MAX_NOTES] = {0};
    float active_amp[MAX_NOTES] = {0};
    int active_count = 0;

    for (int i = MAX_TONE_PIANO-1; i >= 0 && active_count < MAX_NOTES; i--) {
#if 0
        if (note_piano[i] == 1) {
            decay_tick[i] = HAL_GetTick();
        }
#else
        if (note_piano[i] == 1) {
            if (!tone_state[i]) {
                tone_state[i] = 1;
                decay_tick[i] = HAL_GetTick();
            }
        }
        else {
            if (tone_state[i]) {
                tone_state[i] = 0;
            }
        }
#endif
        if (HAL_GetTick() - decay_tick[i] < decay_time) {
            active_amp[active_count] = (float)(decay_time - (HAL_GetTick() - decay_tick[i])) / (float)decay_time;
            active_freqs[active_count] = note_freq(60 + i - 12*0); // MIDI note: C4 = 60
            active_count++;
        }
    }

    float vq_ref = 0, vd_ref = 0;

    if (active_count == 0) {
    	vd_ref = 0;
    	vq_ref = 0;
    } else {
        // Calculate each note's phase and sum them
        float sum = 0.0f;

        for (int i = 0; i < active_count; i++) {
#if 1
            // Use phase accumulation for better frequency stability
            float phase = fmodf(TWO_PI * active_freqs[i] * t, TWO_PI);
            sum += fast_sin(phase) * active_amp[i];
#else

            // sawtooth wave
            sum += (fmodf(active_freqs[i] * t, 1.0f) * 2.0f - 1.0f) * active_amp[i];
#endif
        }

    	vq_ref = 0.0f;
        vd_ref = sum*2.0f;
    }

    v_tone = vd_ref;
    open_loop_voltage_control(hfoc, vd_ref, vq_ref, 0);//hfoc->e_angle_rad_comp

    sample_count++;
    if (sample_count >= (1000 * AUDIO_SAMPLE_RATE)) {
        sample_count = 0;
    }
}
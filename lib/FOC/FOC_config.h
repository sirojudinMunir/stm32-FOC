#ifndef FOC_CONFIG_H_
#define FOC_CONFIG_H_

#define BLDC_PWM_FREQ 20000
#define SPEED_CONTROL_CYCLE	5
#define POSITION_CONTROL_CYCLE 10

#define FOC_TS (1.0f / (float)BLDC_PWM_FREQ)
#define SPEED_TS (FOC_TS * SPEED_CONTROL_CYCLE) 
#define POSITION_TS (FOC_TS * POSITION_CONTROL_CYCLE)

#define MAX_I_SAMPLE 128

#define MAX_SAMPLE_BUFF 1024

#define DEBUG_HFI	0

#define HFI_NEW 1

/* HFI amplitudo (V) */
#define HFI_AMP 3.0f

/* HFI frequency (Hz) */
#define HFI_FREQ 2000.0f

/* LPF Cut-off frequency for Id and Iq */
#define HFI_ID_LPF_FC 200.0f
#define HFI_IQ_LPF_FC 200.0f

/* LPF Cut-off frequency for Ialpha and Ibeta (just for hfi_lpf) */
#define HFI_I_ALPHA_BETA_LPF_FC 200.0f

/* HFI to SMO speed threshold (RPM) */
#define HFI_TO_SMO_THRESHOLD 400.0f
#define SMO_TO_HFI_THRESHOLD 300.0f

/* Polarity Detection Parameters */
#define PD_V_PULSE 4.0f
#define PD_PULSE_TIME 10
#define PD_WAITING_TIME 20

/* Magnetic Encoder Callibration Parameters */
#define CAL_ITERATION 100
#define VD_CAL 0.6f
#define VQ_CAL 0.0f


#endif
#ifndef FOC_CONFIG_H_
#define FOC_CONFIG_H_

#define MAX_I_SAMPLE 128

#define MAX_SAMPLE_BUFF 1024

#define DEBUG_HFI	0

#define HFI_NEW 1

/* HFI amplitudo (V) */
#define HFI_AMP 2.0f

/* HFI frequency (Hz) */
#define HFI_FREQ 1000.0f

/* LPF Cut-off frequency for Id and Iq */
#define HFI_ID_LPF_FC 1000.0f
#define HFI_IQ_LPF_FC 1000.0f

/* LPF Cut-off frequency for Ialpha and Ibeta (just for hfi_lpf) */
#define HFI_I_ALPHA_BETA_LPF_FC 200.0f

/* HFI to SMO speed threshold (RPM) */
#define HFI_TO_SMO_THRESHOLD 450.0f

#endif
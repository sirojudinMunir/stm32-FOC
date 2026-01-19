/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "AS5047P.h"
#include "DRV8302.h"
#include "FOC_utils.h"
#include "bldc_midi.h"
#include "flash.h"
#include "controller_app.h"
#include "CAN.h"
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// #define BLDC_PWM_FREQ AUDIO_SAMPLE_RATE
#define BLDC_PWM_FREQ 10000
#define R_SHUNT	0.01f
#define V_OFFSET_A	1.645f
#define V_OFFSET_B	1.657f
#define POLE_PAIR	(7)
#define SPEED_CONTROL_CYCLE	10
#define FOC_TS (1.0f / (float)BLDC_PWM_FREQ)


#define SENSORLESS_MODE

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;

osThreadId controlTaskHandle;
osThreadId comTaskHandle;
/* USER CODE BEGIN PV */

motor_config_t m_config;
foc_t hfoc;
smo_t hsmo;
DRV8302_t bldc;
AS5047P_t hencd;

float angle_deg = 0.0f;
float sp_input = 0.0f;
int start_cal = 0;
_Bool com_init_flag = 0;
_Bool calibration_flag = 0;
_Bool pos_demo_flag = 0;


float raw_encd_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
void StartControlTask(void const * argument);
void StartComTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************************************************************/

void bldc_init(void) {
  DRV8302_GPIO_MPWM_config(&bldc, M_PWM_GPIO_Port, M_PWM_Pin);
  DRV8302_GPIO_MOC_config(&bldc, M_OC_GPIO_Port, M_OC_Pin);
  DRV8302_GPIO_GAIN_config(&bldc, GAIN_GPIO_Port, GAIN_Pin);
  DRV8302_GPIO_DCCAL_config(&bldc, DC_CAL_GPIO_Port, DC_CAL_Pin);
  DRV8302_GPIO_OCTW_config(&bldc, OCTW_GPIO_Port, OCTW_Pin);
  DRV8302_GPIO_FAULT_config(&bldc, FAULT_GPIO_Port, FAULT_Pin);
  DRV8302_GPIO_ENGATE_config(&bldc, EN_GATE_GPIO_Port, EN_GATE_Pin);
  DRV8302_TIMER_config(&bldc, &htim1, BLDC_PWM_FREQ);
  DRV8302_current_sens_config(&bldc, _10VPV, R_SHUNT, V_OFFSET_A, V_OFFSET_B);
  DRV8302_set_mode(&bldc, _6_PWM_MODE, _CYCLE_MODE);

  DRV8302_init(&bldc);

  DRV8302_disable_dc_cal(&bldc);
  DRV8302_enable_gate(&bldc);
  
}

void control_init(void) {
  hfoc.angle_filtered = &hencd.angle_filtered;
  foc_set_torque_control_bandwidth(&hfoc, m_config.I_ctrl_bandwidth);

  hfoc.id_ctrl.out_max_dynamic = m_config.id_out_max;
  hfoc.iq_ctrl.out_max_dynamic = m_config.iq_out_max;
  pid_init(&hfoc.id_ctrl, m_config.id_kp, m_config.id_ki, 0.0f, FOC_TS, 10.0f, m_config.id_e_deadband);
  pid_init(&hfoc.iq_ctrl, m_config.iq_kp, m_config.iq_ki, 0.0f, FOC_TS, 10.0f, m_config.iq_e_deadband);

  pid_init(&hfoc.speed_ctrl, m_config.speed_kp, m_config.speed_ki, 0.0001f, FOC_TS * SPEED_CONTROL_CYCLE, m_config.speed_out_max, m_config.speed_e_deadband);
  hfoc.speed_ctrl.d_alpha_filter = 0.01f;
  pid_init(&hfoc.pos_ctrl, m_config.pos_kp, m_config.pos_ki, m_config.pos_kd, FOC_TS * SPEED_CONTROL_CYCLE * 10, m_config.pos_out_max, m_config.pos_e_deadband);
  hfoc.pos_ctrl.d_alpha_filter = 0.85;
  
  foc_pwm_init(&hfoc, &(TIM1->CCR3), &(TIM1->CCR2), &(TIM1->CCR1), bldc.pwm_resolution);
  foc_motor_init(&hfoc, POLE_PAIR, 360);

  foc_sensor_init(&hfoc, m_config.encd_offset, REVERSE_DIR);
  foc_gear_reducer_init(&hfoc, 1.0f);
  foc_set_limit_current(&hfoc, 20.0);
  
  hfoc.Ld = m_config.Ld;
  hfoc.Lq = m_config.Lq;
  float Rs = m_config.Rs;
  float Ls = (hfoc.Ld + hfoc.Lq) / 2.0f;
  smo_init(&hfoc.smo, Rs, Ls, POLE_PAIR, FOC_TS);

  // HFI parameter
  foc_sensorless_init(&hfoc, BLDC_PWM_FREQ);
}

void magnetic_encoder_init(void) {
  AS5047P_config(&hencd, &hspi1, SPI_CS_GPIO_Port, SPI_CS_Pin);
  AS5047P_start(&hencd);
  HAL_Delay(10);
  AS5047P_start(&hencd);
}

/******************************************************************************/

float get_power_voltage(void) {
	static float pv_filtered = 0.0f;
  const float filter_alpha = 0.2f; 

	// convert to volt
	float pv = (float)ADC3->JDR1 * ADC_2_POWER_VOLT;

  // Low-pass filter for noise reduction
	pv_filtered = (1.0f - filter_alpha) * pv_filtered + filter_alpha * pv;

	return pv_filtered;
}

void get_v_phase(foc_t *hfoc) {
  const float filter_alpha = 0.98f; 

  float va = (float)ADC3->JDR2 * ADC_2_POWER_VOLT * 0.810810811;
  float vb = (float)ADC3->JDR3 * ADC_2_POWER_VOLT * 0.625;
  float vc = (float)ADC3->JDR4 * ADC_2_POWER_VOLT * 0.769230769;

  hfoc->va = (1.0f - filter_alpha) * hfoc->va + filter_alpha * va;
  hfoc->vb = (1.0f - filter_alpha) * hfoc->vb + filter_alpha * vb;
  hfoc->vc = (1.0f - filter_alpha) * hfoc->vc + filter_alpha * vc;
}

/******************************************************************************/

uint32_t get_dt_us(void) {
#if 0
  static uint32_t last_us = 0;

  uint32_t now_us = TIM10->CNT;
  uint32_t elapsed_us = now_us - last_us;
  last_us = now_us;
#else
  uint32_t elapsed_us = TIM10->CNT;
  TIM10->CNT = 0;
#endif
  return elapsed_us;
}

/******************************************************************************/

void start_measure(inject_taregt_t target) {
  hfoc.meas_inj_target = target;
  osDelay(100);
  hfoc.meas_inj_start_flag = 1;
  // waiting process
  while(hfoc.meas_inj_start_flag) {
    osDelay(1);
  }
}

int measure_R(float vdc) {
  if (hfoc.v_bus  < 10.0f) {
    return -1;
  }

  hfoc.meas_inj_amp = vdc;

  memset(Vd_buff, 0, sizeof(Vd_buff));
  memset(Id_buff, 0, sizeof(Id_buff));

  start_measure(RS);
  estimate_resistance(&hfoc);
  m_config.Rs = hfoc.Rs;

  usb_print("Estimate Resistance @(V=%.2f)\r\n"
            "Rs: %f\r\n\r\n",
            hfoc.meas_inj_amp, hfoc.Rs);
  
  return 0;
}

int measure_L(float f, float amp) {
  if (hfoc.v_bus  < 10.0f) {
    return -1;
  }

  hfoc.meas_inj_freq = f;
  hfoc.meas_inj_amp = amp;
  hfoc.meas_inj_omega = TWO_PI * hfoc.meas_inj_freq;

  memset(Vd_buff, 0, sizeof(Vd_buff));
  memset(Id_buff, 0, sizeof(Id_buff));
  memset(Vq_buff, 0, sizeof(Vq_buff));
  memset(Iq_buff, 0, sizeof(Iq_buff));

  start_measure(LD);
  start_measure(LQ);
  estimate_inductance(&hfoc, FOC_TS);
  m_config.Ld = hfoc.Ld;
  m_config.Lq = hfoc.Lq;

  for (int i = 0; i < MAX_I_SAMPLE; i++) {
    float buffer_val[4] = {
      Vd_buff[i], Vq_buff[i],
      Id_buff[i], Iq_buff[i]
    };
    send_data_float(buffer_val, 4);
    osDelay(1);
  }
  
  usb_print("Estimate Inductance @(f=%.2fHz)\r\n"
            "Ld: %f\r\n"
            "Lq: %f\r\n\r\n", 
            hfoc.meas_inj_freq, hfoc.Ld, hfoc.Lq);

  return 0;
}

/******************************************************************************/

_Bool test_bw_flag = 0;
_Bool start_test_bw = 0;

void test_bandwidth(void) {
  start_test_bw = 1;
  hfoc.id_ref = 1.0f;
  hfoc.iq_ref = 0.0f;

  while (start_test_bw) {
    osDelay(1);
  }
  
  for (int i = 0; i < MAX_I_SAMPLE; i++) {
#if 0
    send_data_float(&Id_buff[i], 4);
#else
    char usb_buff[64];
    snprintf(usb_buff, sizeof(usb_buff), "%f\r\n", Id_buff[i]);
    CDC_Transmit_FS((uint8_t*)usb_buff, sizeof(usb_buff));
#endif
    osDelay(1);
  }
  
  hfoc.id_ref = 0.0f;
  hfoc.iq_ref = 0.0f;
}

void test_bw_get_sample(void) {
  static int s = 0;
  if (start_test_bw) {
    Id_buff[s] = hfoc.id_filtered;
    s++;
    if (s >= MAX_I_SAMPLE) {
      s = 0;
      start_test_bw = 0;
    }
  }
}

/******************************************************************************/

void test_openloop(void) {
  open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, 0);
  HAL_Delay(2000);

  for (float i = 0; i < 2*PI*hfoc.pole_pairs; i+=0.1) {
    open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, i);
    HAL_Delay(1);
  }
  open_loop_voltage_control(&hfoc, 0, 0, 0);
}

/******************************************************************************/


// platformio run --target upload 
// platformio run --target clean
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  MX_USB_DEVICE_Init();

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  printf("FPU aktif!\n");
#else
  printf("FPU tidak aktif!\n");
#endif

  flash_read_config(&m_config);
  init_trig_lut();
  bldc_init();
  control_init();
  CAN_init(&hcan1);
#ifndef SENSORLESS_MODE
  magnetic_encoder_init();
#endif
	// current sensor
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	// voltage sensor
	HAL_ADCEx_InjectedStart_IT(&hadc3);

  HAL_TIM_Base_Start(&htim10);

  // anti-shock at startup
  hfoc.control_mode = POWER_UP_MODE;

  for (int i = 0; i < 5; i++) {
    LED_GPIO_Port->BSRR = LED_Pin;
    HAL_Delay(50);
    LED_GPIO_Port->BSRR = LED_Pin<<16;
    HAL_Delay(50);
  }
  HAL_Delay(500);
  // default mode
  hfoc.control_mode = TORQUE_CONTROL_MODE;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityHigh, 0, 512);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of comTask */
  osThreadDef(comTask, StartComTask, osPriorityLow, 0, 512);
  comTaskHandle = osThreadCreate(osThread(comTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if (HSE_VALUE == 16000000U)
  RCC_OscInitStruct.PLL.PLLM = 8;
#else
  RCC_OscInitStruct.PLL.PLLM = 6;
#endif
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
#if 0
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = 4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1024;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 5;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|DC_CAL_Pin|GAIN_Pin|M_OC_Pin
                          |M_PWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin DC_CAL_Pin GAIN_Pin M_OC_Pin
                           M_PWM_Pin */
  GPIO_InitStruct.Pin = LED_Pin|DC_CAL_Pin|GAIN_Pin|M_OC_Pin
                          |M_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTW_Pin FAULT_Pin */
  GPIO_InitStruct.Pin = OCTW_Pin|FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_GATE_Pin */
  GPIO_InitStruct.Pin = EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static int torque_control_update(void) {
  int ret = 0;
  static uint8_t event_speed_loop_count = 0;
  DRV8302_get_current(&bldc, &hfoc.ia, &hfoc.ib);

#ifdef SENSORLESS_MODE
  foc_sensorless_current_control_update(&hfoc, FOC_TS);

  // calc mechanical angle
  static float last_e_rad = 0.0f;
  static float rad_ovf = 0.0f;
  float angle_diff = hfoc.e_rad - last_e_rad;
  last_e_rad = hfoc.e_rad;

  if (angle_diff < -PI) {
    rad_ovf += 1.0f;
  }
  else if (angle_diff > PI) {
    rad_ovf -= 1.0f;
  }
  float total_e_angle = hfoc.e_rad + rad_ovf * TWO_PI;
  float mechanical_angle_deg = RAD_TO_DEG(total_e_angle) / POLE_PAIR;
  hfoc.actual_angle = mechanical_angle_deg;

  if (event_speed_loop_count >= SPEED_CONTROL_CYCLE) {
    // hfoc.actual_rpm = (rpm_temp / (float)SPEED_CONTROL_CYCLE);
    // rpm_temp = 0;
    event_speed_loop_count = 0;
    foc_set_flag();
    ret = 1;
  }
  event_speed_loop_count++;
#else
  hfoc.e_rad = hfoc.e_angle_rad_comp;
  foc_current_control_update(&hfoc);

  if (event_speed_loop_count > SPEED_CONTROL_CYCLE) {
    event_speed_loop_count = 0;
    uint32_t dt_us = get_dt_us();
    float rpm_encd = AS5047P_get_rpm(&hencd, dt_us);
    foc_calc_mech_rpm_encoder(&hfoc, rpm_encd);
    foc_set_flag();
    ret = 1;
  }
  event_speed_loop_count++;
#endif
  return ret;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
    angle_deg = AS5047P_get_degree(&hencd);
    foc_calc_electric_angle(&hfoc, DEG_TO_RAD(angle_deg));
    AS5047P_set_val_flag();
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
    DRV8302_set_adc_a(&bldc, ADC1->JDR1);
  }
	if (hadc->Instance == ADC2) {
    DRV8302_set_adc_b(&bldc, ADC2->JDR1);
  }
	if (hadc->Instance == ADC3) {

#ifndef SENSORLESS_MODE
    if (AS5047P_get_val_flag()) {
      AS5047P_reset_val_flag();
      AS5047P_start(&hencd);
    }
#endif

    hfoc.v_bus = get_power_voltage();
    // get_v_phase(&hfoc);
    
    switch(hfoc.control_mode) {
      case TORQUE_CONTROL_MODE: {
        hfoc.id_ref = 0.0f;
        hfoc.iq_ref = sp_input;
        torque_control_update();
        break;
      }
      case SPEED_CONTROL_MODE: {
        if (torque_control_update() == 1) {
          static float sp_rpm = 0.0f;
          const float acc_rpm = 1.0f;

          if (sp_input > sp_rpm) {
            sp_rpm += acc_rpm;
          }
          else if (sp_input < sp_rpm) {
            sp_rpm -= acc_rpm;
          }
          foc_speed_control_update(&hfoc, sp_rpm);
        }
        break;
      }
      case POSITION_CONTROL_MODE: {
        if (torque_control_update() == 1) {
#ifndef SENSORLESS_MODE
          float deg_encd = AS5047P_get_actual_degree(&hencd);
          foc_calc_mech_pos_encoder(&hfoc, deg_encd);
#endif
          foc_position_control_update(&hfoc, sp_input);
        }
        break;
      }
      case AUDIO_MODE: {
        audio_loop(&hfoc);
        break;
      }
      case CALIBRATION_MODE: {
        DRV8302_get_current(&bldc, &hfoc.ia, &hfoc.ib);
        meas_inj_dq_process(&hfoc, FOC_TS);
        break;
      }
      case TEST_MODE: {
        DRV8302_get_current(&bldc, &hfoc.ia, &hfoc.ib);
        foc_current_control_update(&hfoc);
        test_bw_get_sample();
        break;
      }
      case POWER_UP_MODE:
        open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
      break;
      default:
      break;
    }
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the controlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t blink_tick = 0;
  float pos_demo[] = {
    -10.0f, 0.0f,

    // -180.0f, 0.0f,
    // -180.0f, 0.0f,
    // -180.0f, 0.0f,

    // -45.0f, -90.0f, -135.0f, -180.0f,
    // -135.0f, -90.0f, - 45.0f, 0.0f
  };
  int num_elements = sizeof(pos_demo) / sizeof(pos_demo[0]);
  uint32_t test_cycle = 0;
  /* Infinite loop */
  for(;;)
  { 
    if (hfoc.control_mode == CALIBRATION_MODE) {
      if (start_cal == 1) {
#ifndef SENSORLESS_MODE
        foc_cal_encoder(&hfoc);
#endif

        measure_R(1.0f);
        measure_L(1000.0f, 1.2f);

#ifdef SENSORLESS_MODE
        smo_update_R_L(&hsmo, m_config.Rs, (m_config.Ld + m_config.Lq) * 0.5);
#endif
        flash_auto_tuning_torque_control(&m_config);
        hfoc.id_ctrl.kp = m_config.id_kp;
        hfoc.id_ctrl.ki = m_config.id_ki;
        hfoc.iq_ctrl.kp = m_config.iq_kp;
        hfoc.iq_ctrl.ki = m_config.iq_ki;

        start_cal = 0;
      }
    }
    else if (hfoc.control_mode == TEST_MODE) {
      if (test_bw_flag) {
        test_bandwidth();
        test_bw_flag = 0;
      }
    }
    // actuator demo:
    if (pos_demo_flag && hfoc.control_mode == POSITION_CONTROL_MODE) {
      static int pos_index = 0;

      sp_input = pos_demo[pos_index];
      float error_angle = hfoc.actual_angle - pos_demo[pos_index]; 
      if (fabs(error_angle) < 0.5f) {
        pos_index++;
        if (pos_index >= num_elements) {
          pos_index = 0;
          test_cycle++;
          if (test_cycle > 30) {
            test_cycle = 0;
            // pos_demo_flag = 0;
          }
        }
      }
    }

    if (HAL_GetTick() - blink_tick >= 500) {
      blink_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartComTask */
/**
* @brief Function implementing the comTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComTask */
void StartComTask(void const * argument)
{
  /* USER CODE BEGIN StartComTask */

  uint32_t debug_tick = HAL_GetTick();
  uint32_t transmit_tick = HAL_GetTick();

  motor_mode_t last_mode = CALIBRATION_MODE;

  /* Infinite loop */
  for(;;)
  {
#if DEBUG_HFI
    static int sample_index = 0;
    if (hfoc.collect_sample_flag) {
      float data[16];
      uint16_t len = 0;

      if (sample_index == 0) {
        erase_graph(); 
        osDelay(1);
      }

      data[len++] = i_alpha_buff[sample_index];
      data[len++] = i_beta_buff[sample_index];
      data[len++] = i_alpha_l_buff[sample_index];
      data[len++] = i_beta_l_buff[sample_index];
      send_data_float(data, len);
      sample_index++;
      if (sample_index > MAX_SAMPLE_BUFF) {
        osDelay(1000);
        sample_index = 0;
        hfoc.collect_sample_flag = 0;
      }
    }
#else
    if (HAL_GetTick() - debug_tick >= 2) {
      debug_tick = HAL_GetTick();
      float data[16];
      uint16_t len = 0;

      switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        data[len++] = sp_input;
        data[len++] = hfoc.id_filtered;
        data[len++] = hfoc.iq_filtered;
        data[len++] = hfoc.e_rad;
        data[len++] = hfoc.e_angle_rad_comp;
        break;
      case SPEED_CONTROL_MODE:
        data[len++] = sp_input;
#ifdef SENSORLESS_MODE
        if (hfoc.state == MOTOR_STATE_HFI) {
          data[len++] = hfoc.actual_rpm;
          data[len++] = 0.0f;
        }
        else {
          data[len++] = 0.0f;
          data[len++] = hfoc.actual_rpm;
        }
#else
        data[len++] = hfoc.actual_rpm;
#endif
        break;
      case POSITION_CONTROL_MODE:
        data[len++] = sp_input;
        data[len++] = hfoc.actual_angle;
        break;
      case AUDIO_MODE:
        data[len++] = v_tone;
        break;
      case CALIBRATION_MODE:
#ifndef SENSORLESS_MODE
        data[len++] = hencd.prev_raw_angle;
        data[len++] = hencd.angle_filtered;
#endif
        break;
      case TEST_MODE:
        // data[len++] = hfoc.v_bus;
        data[len++] = hfoc.ia;
        data[len++] = hfoc.ib;
        data[len++] = hfoc.actual_angle;
        break;
      default:
      break;
      }

      send_data_float(data, len);
    }
#endif

    if (com_init_flag) {
      last_mode = -1;
      com_init_flag = 0;
    }

    if (calibration_flag) {
      last_mode = -1;
      calibration_flag = 0;
    }

    if (hfoc.control_mode != last_mode) {
      last_mode = hfoc.control_mode;
      float data[16];

      osDelay(2);
      erase_graph(); osDelay(1);
      switch (hfoc.control_mode) {
      case TORQUE_CONTROL_MODE:
        send_data_float(data, 3); osDelay(1);
        change_title("CURRENT CONTROL MODE"); osDelay(1);
        change_legend(0, "Iq_sp"); osDelay(1);
        change_legend(1, "Id"); osDelay(1);
        change_legend(2, "Iq"); osDelay(1);
        break;
      case SPEED_CONTROL_MODE:
#ifdef SENSORLESS_MODE
        send_data_float(data, 3); osDelay(1);
        change_title("SPEED CONTROL MODE"); osDelay(1);
        change_legend(0, "set point"); osDelay(1);
        change_legend(1, "rpm_hfi"); osDelay(1);
        change_legend(2, "rpm_smo"); osDelay(1);
#else
        send_data_float(data, 2); osDelay(1);
        change_title("SPEED CONTROL MODE"); osDelay(1);
        change_legend(0, "set point"); osDelay(1);
        change_legend(1, "rpm"); osDelay(1);
#endif
        break;
      case POSITION_CONTROL_MODE:
        send_data_float(data, 2); osDelay(1);
        change_title("POSITION CONTROL MODE"); osDelay(1);
        change_legend(0, "set point"); osDelay(1);
        change_legend(1, "actual angle"); osDelay(1);
        break;
      case CALIBRATION_MODE:
        // send_data_float(data, 1); osDelay(1);
        // change_title("CALIBRATION MODE"); osDelay(1);
        // change_legend(0, "error angle (rad)"); osDelay(1);
        send_data_float(data, 4); osDelay(1);
        change_title("CALIBRATION MODE"); osDelay(1);
        change_legend(0, "Vd"); osDelay(1);
        change_legend(1, "Vq"); osDelay(1);
        change_legend(2, "Id"); osDelay(1);
        change_legend(3, "Iq"); osDelay(1);
        start_cal = 1;
        break;
      case AUDIO_MODE:
        send_data_float(data, 1); osDelay(1);
        change_title("AUDIO MODE"); osDelay(1);
        change_legend(0, "vd"); osDelay(1);
        break;
      case TEST_MODE:
        send_data_float(data, 3); osDelay(1);
        change_title("TEST MODE"); osDelay(1);
        change_legend(0, "Ia"); osDelay(1);
        change_legend(1, "Ib"); osDelay(1);
        change_legend(1, "angle"); osDelay(1);
        break;
      default:
        break;
      }
    }

    if (HAL_GetTick() - transmit_tick >= 10) {
      transmit_tick = HAL_GetTick();
      uint8_t can_tx_buff[5];
      union { float f; uint8_t b[4]; } u;
      u.f = hfoc.actual_angle;
      can_tx_buff[0] = 0x30;
      can_tx_buff[1] = u.b[0];
      can_tx_buff[2] = u.b[1];
      can_tx_buff[3] = u.b[2];
      can_tx_buff[4] = u.b[3];
      CAN_Send(&hcan1, TRANSMITTER_ID, can_tx_buff, 5);
    }
    osDelay(1);
  }
  /* USER CODE END StartComTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  DRV8302_disable_gate(&bldc);
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

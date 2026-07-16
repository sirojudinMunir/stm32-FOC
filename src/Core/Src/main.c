/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f405_link.h"
#include "CAN.h"
#include "motor.h"
#include "AS5047P.h"
#include "FOC_utils.h"
#include "self_commissioning.h"
#include "storage.h"
#include "com.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define POLE_PAIR	(7)
#define ENCODER_OFFSET_RAD (DEG_TO_RAD(297.242249f))

#define usb_print(fmt, ...)                                                 \
  do {                                                                      \
    snprintf(usb_send_buff, sizeof(usb_send_buff), fmt, ##__VA_ARGS__);     \
    CDC_Transmit_FS((uint8_t *)(usb_send_buff), sizeof(usb_send_buff) - 1); \
  } while (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

AS5047P_t hencd1;
uint32_t adc_buff[4];
foc_t hfoc1;
self_commissioning_t hsc1;
storage_t hstorage1;
com_t husb_com;
com_t hcan_com;

char usb_send_buff[128];
float data_plotter[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**************************************************************************** */
// implementation

void motor1_inverter_enable(void) {
  EN_GATE1_GPIO_Port->BSRR = EN_GATE1_Pin;
  link_enable_pwm(&htim1);
  link_enable_adc(&hadc1);
}

void motor1_inverter_disable(void) {
  EN_GATE1_GPIO_Port->BSRR = EN_GATE1_Pin<<16;
  link_disable_pwm(&htim1);
  link_disable_adc(&hadc1);
}

uint32_t motor1_get_pwm_res(void) {
  return link_get_max_pwm(&htim1);
}

float motor1_as5047p_get_mech_deg(void) {
  return AS5047P_get_degree(&hencd1);
}

float motor1_as5047p_get_rpm(void) {
  return AS5047P_get_rpm(&hencd1, FOC_TS);
}

int motor1_as5047p_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len) {
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, len) == HAL_OK) return 0;
  return -1;
}

void motor1_as5047p_spi_cs(_Bool cs_state) {
  if (cs_state) SPI_CS_GPIO_Port->BSRR = SPI_CS_Pin;
  else SPI_CS_GPIO_Port->BSRR = SPI_CS_Pin<<16;
}


#define FLASH_SECTOR_ADDR  ((uint32_t)0x080E0000)
#define FLASH_SECTOR_NUM   FLASH_SECTOR_11

int write_flash(void *data, uint32_t len) {
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  
  HAL_FLASH_Unlock();

  EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector       = FLASH_SECTOR_NUM;
  EraseInitStruct.NbSectors    = 1;

  status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    return -1;
  }

  uint32_t address = FLASH_SECTOR_ADDR;
  uint8_t *src = (uint8_t *)data;

  for (uint32_t i = 0; i < len; i += 4) {
    uint32_t word = *(uint32_t*)(src + i);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word);
    if (status != HAL_OK) {
      HAL_FLASH_Lock();
      return -1;
    }
    address += 4;
  }

  HAL_FLASH_Lock();
  return 0;
}

int read_flash(void *data, uint32_t len) {
  memcpy(data, (void*)FLASH_SECTOR_ADDR, len);
  return 0;
}

int usb_recv_data(uint8_t *data, uint16_t len) {
  (void)data;
  (void)len;
  return 0;
}

int usb_send_data(uint8_t *data, uint16_t len) {
  if (CDC_Transmit_FS(data, len) != USBD_OK) return -1;
  return 0;
}

int can_recv_data(uint8_t *data, uint16_t len) {
  (void)data;
  (void)len;
  return 0;
}

int can_send_data(uint8_t *data, uint16_t len) {
#if USB_TO_CAN
  CAN_Send(&hcan1, 0x05, data, len);
#else
  CAN_Send(&hcan1, TRANSMITTER_ID, data, len);
#endif
  return 0;
}

/**************************************************************************** */

static void init_motor(void) {
  motor_init_pwm(&hfoc1.motor, &TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3);
  motor_init_adc_current_sense(&hfoc1.motor, &adc_buff[0], &adc_buff[1], &adc_buff[2]);
  motor_set_current_sense_gain(&hfoc1.motor, (3.3f / 4095.0f) / (0.01f * 20.0f));
}

static void init_encoder(void) {
  AS5047P_spi_config(&hencd1, motor1_as5047p_spi_transmit, motor1_as5047p_spi_cs);
  AS5047P_init(&hencd1, SENSOR_DIR_REVERSE, (360.0f / 16383.0f));
  AS5047P_set_angle_filter_fc(&hencd1, 1000.0f, FOC_TS);
  AS5047P_set_rpm_filter_fc(&hencd1, 50.0f, FOC_TS);
}

static void init_foc(void) {
  CAN_init(&hcan1);
#if USB_TO_CAN
  com_init(&husb_com, usb_recv_data, usb_send_data, HAL_GetTick, &hfoc1, &hstorage1, &hsc1);
  com_init(&hcan_com, can_recv_data, can_send_data, HAL_GetTick, &hfoc1, &hstorage1, &hsc1);
#else
  init_trig_lut();
  link_set_pwm_freq(&htim1, BLDC_PWM_FREQ);
  init_motor();
  init_encoder();
  sc_init(&hsc1, &hfoc1);

  storage_init(&hstorage1, write_flash, read_flash);
  foc_inverter_init(&hfoc1, motor1_inverter_enable, motor1_inverter_disable, motor1_get_pwm_res);
  foc_feedback_sensor_init(&hfoc1, motor1_as5047p_get_mech_deg, motor1_as5047p_get_rpm);
  com_init(&husb_com, usb_recv_data, usb_send_data, HAL_GetTick, &hfoc1, &hstorage1, &hsc1);
  com_init(&hcan_com, can_recv_data, can_send_data, HAL_GetTick, &hfoc1, &hstorage1, &hsc1);

  storage_read_config(&hstorage1);
  storage_copy_to_local(&hstorage1, &hfoc1);

  // Id PI parameter
  pid_reset(&hfoc1.id_ctrl);
  pid_set_ts(&hfoc1.id_ctrl, FOC_TS);
  // Id PI parameter
  pid_reset(&hfoc1.iq_ctrl);
  pid_set_ts(&hfoc1.iq_ctrl, FOC_TS);
  // Speed PID parameter
  pid_reset(&hfoc1.speed_ctrl);
  pid_set_ts(&hfoc1.speed_ctrl, SPEED_TS);
  pid_set_kd(&hfoc1.speed_ctrl, 0);
  pid_set_d_filter_fc(&hfoc1.speed_ctrl, 100.0f);
  pid_set_max_d(&hfoc1.speed_ctrl, 10.0f);
  // Position PID parameter
  pid_reset(&hfoc1.pos_ctrl);
  pid_set_ts(&hfoc1.pos_ctrl, POSITION_TS);
  pid_set_d_filter_fc(&hfoc1.pos_ctrl, 20.0f);
  pid_set_max_d(&hfoc1.pos_ctrl, 100.0f);
  // field weakening
  pid_reset(&hfoc1.fw_ctrl);
  pid_set_ts(&hfoc1.fw_ctrl, FOC_TS);

  foc_motor_init(&hfoc1, POLE_PAIR, 360.0f);

  // foc_set_mode(&hfoc1, FOC_MODE_SENSORLESS_SMO_HFI_NEW);
  foc_sensorless_init(&hfoc1, BLDC_PWM_FREQ);

  foc_sensor_init(&hfoc1, ENCODER_OFFSET_RAD, NORMAL_DIR);
  foc_gear_reducer_init(&hfoc1, 1.0f);
  foc_set_limit_current(&hfoc1, 10.0f);

  hfoc1.v_bus = 12.0f; 

  foc_enable(&hfoc1);
#endif
}

static void indicator_update(void) {
  static uint32_t heartbeat_tick = 0;
  if (HAL_GetTick() - heartbeat_tick >= 500) {
    heartbeat_tick = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  }
  if (HAL_GPIO_ReadPin(NFAULT1_GPIO_Port, NFAULT1_Pin)) {
    LED_R_GPIO_Port->BSRR = LED_R_Pin<<16;
  }
  else {
    LED_R_GPIO_Port->BSRR = LED_R_Pin;
  }
}

static void self_commissioning_update(void) {
  if (sc_is_measure_done(&hsc1)) {
    switch(sc_get_seq(&hsc1)) {
      case SC_SEQUENCE_START_MEASURE_RS:
        foc_set_motor_Rs(&hfoc1, sc_get_Rs(&hsc1));
        break;
      case SC_SEQUENCE_START_MEASURE_LD:
        foc_set_motor_Ld(&hfoc1, sc_get_Ld(&hsc1));
        break;
      case SC_SEQUENCE_START_MEASURE_LQ:
        foc_set_motor_Lq(&hfoc1, sc_get_Lq(&hsc1));
        break;
    }
  }
}

void usb_to_can_update(void) {
  if (husb_com.incomming_data_flag) {
    husb_com.incomming_data_flag = 0;
    hcan_com.send_data(husb_com.data_rx, husb_com.data_rx_len);
  }
  if (hcan_com.incomming_data_flag) {
    hcan_com.incomming_data_flag = 0;
    husb_com.send_data(hcan_com.data_rx, hcan_com.data_rx_len);
  }
}

/**************************************************************************** */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
    AS5047P_set_spi_transfer_done(&hencd1);
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
    adc_buff[0] = ADC1->JDR1;
    adc_buff[1] = ADC1->JDR2;
    adc_buff[2] = ADC1->JDR3;
    sc_update(&hsc1, FOC_TS);
    foc_update(&hfoc1, FOC_TS);
  }
}

// CAN RX FIFO 0 Callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[32];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    hcan_com.data_rx = RxData;
    hcan_com.data_rx_len = RxHeader.DLC;
    hcan_com.incomming_data_flag = 1;
    // usb_print("(%ld) %d %d %d %d\r\n", hcan_com.data_rx_len, RxData[0], RxData[1], RxData[2], RxData[3]);
  }
}

/**************************************************************************** */

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
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  printf("FPU aktif!\n");
#else
  printf("FPU tidak aktif!\n");
#endif

  init_foc();

  uint32_t com_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    indicator_update();
#if USB_TO_CAN
    usb_to_can_update();
#else
    AS5047P_update(&hencd1);
    com_update(&husb_com);
    com_update(&hcan_com);
    self_commissioning_update();
#endif
    if (HAL_GetTick() - com_tick >= 5) {
      com_tick = HAL_GetTick();
      data_plotter[0] = hfoc1.e_rad;
      data_plotter[1] = hfoc1.motor.ia;
      data_plotter[2] = hfoc1.motor.ib;
      data_plotter[3] = hfoc1.motor.ic;
      com_update_plotter(&husb_com, data_plotter, 4);
      // uint8_t tx_buff[2] = {
      //   0xAA, 0x55
      // };
      // CAN_Send(&hcan1, 0x02, tx_buff, 2);
      // uint8_t usb_tx_buff[128];
      // uint16_t ln = snprintf((char *)usb_tx_buff, sizeof(usb_tx_buff), 
      //                       "%f %ld %ld %ld %f %f %f\r\n", 
      //                       hfoc1.e_rad, 
      //                       TIM1->CCR1, TIM1->CCR2, TIM1->CCR3, 
      //                       hfoc1.motor.ia, hfoc1.motor.ib, hfoc1.motor.ic);
      // uint16_t ln = snprintf((char *)usb_tx_buff, sizeof(usb_tx_buff), 
      //                       "%f %f %f\r\n", 
      //                       hfoc1.e_rad, hfoc1.actual_angle, hfoc1.actual_rpm);
      // CDC_Transmit_FS(usb_tx_buff, ln);
    }


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
  RCC_OscInitStruct.PLL.PLLM = 6;
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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
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

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  sBreakDeadTimeConfig.DeadTime = 25;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
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
  HAL_GPIO_WritePin(GPIOC, LED_R_Pin|LED_G_Pin|EN_GATE1_Pin|SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin EN_GATE1_Pin SPI_CS_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|EN_GATE1_Pin|SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NFAULT1_Pin */
  GPIO_InitStruct.Pin = NFAULT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NFAULT1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

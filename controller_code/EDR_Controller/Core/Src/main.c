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
#include "odrive_can_commands.h"

#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */


uint8_t buffer[1024];
char msg[100];

//-------ASCII commands for Odrive with UART---------
char fullCallib[] = "w axis0.requested_state 3\n";
char en8[] = "w axis0.requested_state 8\n";
char saveConfig[] = "ss";
char trapezoidal[] = "t 0 -2\n";
char motor_velo[] = "v 0 1 0\n";
char read_voltage[] = "r vbus_voltage\n";


uint8_t uart_rx_buf[8];
uint16_t uart_rx_index = 0;

FDCAN_FilterTypeDef filter;

FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;

uint8_t txData[8] =
{
    1,2,3,4,5,6,7,8
};

uint8_t rxData[8];
uint8_t indx;


uint32_t can_id;
uint8_t axis_id=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_RTC_Init(void);
static void MX_UART5_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//  HAL_PWR_EnableBkUpAccess();
//
//  if (__HAL_RCC_GET_RTC_SOURCE() != RCC_RTCCLKSOURCE_LSE) {
//      __HAL_RCC_BACKUPRESET_FORCE();
//      __HAL_RCC_BACKUPRESET_RELEASE();
//  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C4_Init();
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */


//  rcc_cr_snapshot = RCC->CR;
//
//  // HSE ready flag
//  hse_ready = __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY);
//
//  // PLL source check (very important on your config)
//  pll_source = __HAL_RCC_GET_PLL_OSCSOURCE();
//
//  // actual system clock frequency
//  sysclk_hz = HAL_RCC_GetSysClockFreq();



//  -------Configuring a mask filter ---------
//  filter.IdType = FDCAN_STANDARD_ID;
//  filter.FilterIndex = 0;
//  filter.FilterType = FDCAN_FILTER_MASK;
//  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//
//  filter.FilterID1 = 0x000;
//  filter.FilterID2 = 0x000;
//
//
//  HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
//
////  //---------Configuring the TX header
//  txHeader.Identifier = 0x123;
//  txHeader.IdType = FDCAN_STANDARD_ID;
//  txHeader.TxFrameType = FDCAN_DATA_FRAME;
//  txHeader.DataLength = FDCAN_DLC_BYTES_8;
//  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
//  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
//  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//  txHeader.MessageMarker = 0;
////
////------------starting  fdcan2-----------------------------------
//HAL_FDCAN_Start(&hfdcan2);
////
////
////--------Coding RX interrupt-----------
//HAL_FDCAN_ActivateNotification(
//          &hfdcan2,
//          FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
//          0);


//  //-------Debugging with status checks for CAN init
//  HAL_StatusTypeDef ret;
//
//  ret = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
//  sprintf((char*)buffer,"Filter = %d\r\n",ret);
//  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//
//  ret = HAL_FDCAN_Start(&hfdcan1);
//  sprintf((char*)buffer,"Filter = %d\r\n",ret);
//  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//
//  ret = HAL_FDCAN_ActivateNotification(
//		  &hfdcan1,
//		  FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
//		  0);
//  sprintf((char*)buffer, "Notification = %d\r\n",ret);
//  CDC_Transmit_FS(buffer, strlen((char*)buffer));

//----------------UART config for Odrive---------------
//  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, (uint8_t*)en, strlen(en), HAL_MAX_DELAY);
//	 if(status != HAL_OK){
//
//	 }
//  sprintf((char*)buffer,"Starting!");
//  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//

  HAL_UART_Receive_IT(&huart5, uart_rx_buf, 1);
//
//  HAL_StatusTypeDef uart_status = HAL_UART_Transmit(&huart5, (uint8_t*)fullCallib, strlen(fullCallib), 1000);
//  HAL_Delay(10000);
//  HAL_UART_Transmit(&huart5, (uint8_t*)saveConfig, strlen(saveConfig), 1000);
//  HAL_Delay(5000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//----GPIO toggle check-------------------
//
//	  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(1000);

	  /* Read time first */
//	      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//
//	      /* Then read date (required to unlock the shadow registers) */
//	      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
//
//	      sprintf(msg,
//	              "%02d:%02d:%02d\r\n",
//	              sTime.Hours,
//	              sTime.Minutes,
//	              sTime.Seconds);
//
////	      CDC_Transmit_FS((uint8_t*)msg,
////	                          strlen(msg));
//
//	      HAL_Delay(1000);


//-----HSE check------------------------
//	  if(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY))
//	  	  	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4, 1);
//	  else
//		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4, 0);
//	 rcc_cr_snapshot = RCC->CR;
//	 hse_ready = __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY);
//	 sysclk_hz = HAL_RCC_GetSysClockFreq();
//
//	 HAL_Delay(500);

//CDC check
//	  sprintf((char*)buffer,"CDC up and running!\n");
//	  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//	  HAL_Delay(1000);


//---------------CAN---------------------------------

//   for (int i=0; i<8; i++)
//   {
//	txData[i] = indx++;
//   }
//
//   HAL_StatusTypeDef status;
//   status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData);
//   if (status!= HAL_OK)
//   {
//	Error_Handler();
//   }

//-----Clear errors and read bus voltage
//	  uint8_t txData[8] = {0};
//	  uint32_t data =0 ;
//	  memcpy(txData,&data,sizeof(data));
//	  txHeader.Identifier = ODrive_Get_CAN_ID(0, CMD_CLEAR_ERRORS);
//	  txHeader.DataLength = FDCAN_DLC_BYTES_0;
//	  HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData);


//	  HAL_Delay(2000);
//	  txHeader.Identifier = ODrive_Get_CAN_ID(0, CMD_GET_BUS_VOLTAGE_AND_CURRENT);
//	  txHeader.TxFrameType = FDCAN_REMOTE_FRAME;
//	  txHeader.DataLength = FDCAN_DLC_BYTES_8;   // requested length
//	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData);

//	  FDCAN_ProtocolStatusTypeDef status;
//	  HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status);
//	  sprintf((char*)buffer, "LEC:%lu DLEC:%lu EP:%d BO:%d\r\n",
//	          status.LastErrorCode, status.DataLastErrorCode,
//	          status.ErrorPassive, status.BusOff);
//	  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//	  HAL_Delay(2000);

//--closed loop motor control

//	  uint32_t state = 3;
//	  memcpy(txData, &state, sizeof(state));
//	  txHeader.Identifier = ODrive_Get_CAN_ID(0, CMD_SET_AXIS_REQUESTED_STATE);
//	  txHeader.DataLength = FDCAN_DLC_BYTES_4;
//	  HAL_FDCAN_AddMessageToTxFifoQ(
//	          &hfdcan2,
//	          &txHeader,
//	          txData
//	  );
//	  HAL_Delay(6000);

//	  uint8_t data[8] = {0};
//
//	  uint32_t state = 8;   // CLOSED_LOOP_CONTROL
//
//	  memcpy(data, &state, 4);
//
//	  ODrive_CAN_Send(
//	      1,                         // axis_id
//	      CMD_SET_AXIS_NODE_ID,
//	      data,
//	      8
//	  );


//   HAL_Delay (1000);


//------------------Odrive testing-----------------------------

//	  HAL_Delay(500);

//	  // Diagnostic feedback
//	  if(uart_status == HAL_OK) {
//		  sprintf((char*)buffer,"UART TX OK\r\n");
//	  } else if(uart_status == HAL_TIMEOUT) {
//		  sprintf((char*)buffer,"UART TX TIMEOUT\r\n");
//	  } else if(uart_status == HAL_BUSY) {
//		  sprintf((char*)buffer,"UART TX BUSY\r\n");
//	  } else {
//		  sprintf((char*)buffer,"UART TX ERROR: %d\r\n", uart_status);
//	  }
//	  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//	  HAL_Delay(500);


//
	  HAL_UART_Transmit(&huart5, (uint8_t*)read_voltage, strlen(read_voltage), 1000);
	  HAL_Delay(4000);


//	  HAL_UART_Transmit(&huart5, (uint8_t*)motor_velo, strlen(motor_velo), 1000);
//
//	  HAL_Delay(10000);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_UART4;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 3;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 13;
  hfdcan2.Init.NominalTimeSeg1 = 86;
  hfdcan2.Init.NominalTimeSeg2 = 13;
  hfdcan2.Init.DataPrescaler = 5;
  hfdcan2.Init.DataSyncJumpWidth = 10;
  hfdcan2.Init.DataTimeSeg1 = 10;
  hfdcan2.Init.DataTimeSeg2 = 10;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 1;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00000E14;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 11;
  sTime.Minutes = 27;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_Pneu_3_Pin|MCU_Pneu_5_1_Pin|MCU_Pneu_5_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_ODRIVE_I_O_GPIO_Port, MCU_ODRIVE_I_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6
                           PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 PE14
                           PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2
                           PC3 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA8 PA9
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_Pneu_3_Pin MCU_Pneu_5_1_Pin MCU_Pneu_5_2_Pin */
  GPIO_InitStruct.Pin = MCU_Pneu_3_Pin|MCU_Pneu_5_1_Pin|MCU_Pneu_5_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUCK_PG_Pin */
  GPIO_InitStruct.Pin = BUCK_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUCK_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_PIN_Pin */
  GPIO_InitStruct.Pin = LED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB11 PB15
                           PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_ODRIVE_I_O_Pin */
  GPIO_InitStruct.Pin = MCU_ODRIVE_I_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_ODRIVE_I_O_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD14
                           PD15 PD0 PD1 PD2
                           PD3 PD4 PD5 PD6
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//  //---------Callback
//void HAL_FDCAN_RxFifo0Callback(
//        FDCAN_HandleTypeDef *hfdcan,
//        uint32_t RxFifo0ITs)
//{
//
//    if(hfdcan->Instance == FDCAN2)
//    {
//
//        FDCAN_RxHeaderTypeDef RxHeader;
//        uint8_t RxData[8];
//
//
//        HAL_FDCAN_GetRxMessage(
//                hfdcan,
//                FDCAN_RX_FIFO0,
//                &RxHeader,
//                RxData);
//
//
//
////        sprintf(msg,
////        "ODrive RX\r\n"
////        "ID: %lx DATA: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
////        RxHeader.Identifier,
////        RxData[0],
////        RxData[1],
////        RxData[2],
////        RxData[3],
////        RxData[4],
////        RxData[5],
////        RxData[6],
////        RxData[7]);
//       float voltage;
//	   float current;
//
//	   memcpy(&voltage, &RxData[0], 4);
//	   memcpy(&current, &RxData[4], 4);
//
//	   sprintf(msg,
//			   "Voltage = %.2f V\r\n"
//			   "Current = %.2f A\r\n",
//			   voltage,
//			   current);
//
//
//
//        CDC_Transmit_FS(
//                (uint8_t*)msg,
//                strlen(msg));
//
//
//    }
//}
//

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == UART5) {
//
//        uint8_t rx_byte = huart->pRxBuffPtr[0];
//
//        if (uart_rx_index < (sizeof(uart_rx_buf) - 1)) {
//            uart_rx_buf[uart_rx_index++] = rx_byte;
//
//            if (rx_byte == '\n' || rx_byte == '\r') {
//                uart_rx_buf[uart_rx_index] = '\0';
//                CDC_Transmit_FS((uint8_t*)"Loopback RX:\n ", strlen("Loopback RX: "));
//                CDC_Transmit_FS(uart_rx_buf, uart_rx_index);
//                uart_rx_index = 0;
//            } else {
//                CDC_Transmit_FS(&rx_byte, 1);
//            }
//        } else {
//            uart_rx_index = 0;
//        }
//
//        HAL_UART_Receive_IT(&huart5, &uart_rx_buf[uart_rx_index], 1);
//    }
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        // Byte is already in uart_rx_buf[uart_rx_index] (HAL wrote it there)
        uart_rx_index++;

        if (uart_rx_index >= sizeof(uart_rx_buf)) {
            CDC_Transmit_FS(uart_rx_buf, uart_rx_index);
            uart_rx_index = 0;
        }

        // Re-arm for the NEXT byte, every single time
        HAL_UART_Receive_IT(&huart5, &uart_rx_buf[uart_rx_index], 1);
    }
}


uint32_t ODrive_Get_CAN_ID(uint8_t axis_id, uint32_t cmd_id)
{
    return ((uint32_t)axis_id << 5) | cmd_id;
}
//
//void ODrive_CAN_Send(uint8_t axis_id, uint8_t cmd_id, uint8_t *data, uint8_t length)
//{
//    FDCAN_TxHeaderTypeDef TxHeader;
//
//    TxHeader.Identifier = ODrive_Get_CAN_ID(axis_id, cmd_id);
//
//    TxHeader.IdType = FDCAN_STANDARD_ID;
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//    TxHeader.DataLength = length << 16;   // or use FDCAN_DLC_BYTES_x
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    TxHeader.MessageMarker = 0;
//
//    HAL_FDCAN_AddMessageToTxFifoQ(
//        &hfdcan2,
//        &TxHeader,
//        data
//    );
//}




/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

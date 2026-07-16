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
#include "dma.h"
#include "quadspi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "packet_protocol.h"
#include "odrive_can_commands.h"
//#include "scara_app.h"

#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

//Odrive motion
#include <stdarg.h>
#include "app_config.h"
#include "odrive_link.h"
#include "motion.h"
#include "app_log.h"

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

/* USER CODE BEGIN PV */

volatile uint8_t g_control_tick_flag = 0;
static bool s_move_active = false;

uint8_t buffer[1024];
char msg[100];

//-------ASCII commands for Odrive with UART---------
char fullCallib[] = "w axis0.requested_state 3\n";
char closedLoop0[] = "w axis0.requested_state 8\n";
char closedLoop1[] = "w axis1.requested_state 8\n";

char saveConfig[] = "ss";
char trapezoidal[] = "t 0 -2\n";
char motor_velo[] = "v 0 1 0\n";
char read_voltage[] = "r vbus_voltage\n";

uint8_t uart4_rx_buf[8];
uint8_t uart5_rx_buf[8];

uint16_t uart4_rx_index = 0;
uint16_t uart5_rx_index = 0;

//Sample data for transmission from vision controller
uint32_t lastSendTick = 0;
const uint32_t SEND_INTERVAL_MS = 500; /* send a sample packet every 500ms */
int16_t sampleX = 0;
int16_t sampleY = 0;

//FDCAN_FilterTypeDef filter;
//FDCAN_TxHeaderTypeDef txHeader;
//FDCAN_RxHeaderTypeDef rxHeader;
//uint32_t can_id;
//uint8_t axis_id=0;

//uint8_t txData[8] =
//{
//    1,2,3,4,5,6,7,8
//};
//
//uint8_t rxData[8];
//uint8_t indx;
//DISPLAY CHECKING
//static const uint8_t NX_TERM[3] = {0xFF, 0xFF, 0xFF};
///* One-byte-at-a-time RX state machine for Nextion touch events.
// * Packet format: 0x65 <page_id> <component_id> <event> 0xFF 0xFF 0xFF
// * event: 0x01 = press, 0x00 = release. We act on release. */
//static uint8_t nx_rx_byte;            // scratch byte HAL writes into
//static uint8_t nx_pkt[7];             // assembled packet
//static uint8_t nx_pkt_idx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void cdc_log(const char *fmt, ...) /* <-- no 'static' anymore */
{
	char buf[128];
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (len > 0)
		CDC_Transmit_FS((uint8_t*) buf, (uint16_t) len);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		g_control_tick_flag = 1;
	}
}

//--------for display--------------------
//static void nx_send(UART_HandleTypeDef *huart, const char *cmd)
//{
//    HAL_UART_Transmit(huart, (uint8_t *)cmd, strlen(cmd), 100);
//    HAL_UART_Transmit(huart, (uint8_t *)NX_TERM, 3, 100);
//}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

//Adding scara GUI
//  scara_app_init();
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
	MX_DMA_Init();
	MX_QUADSPI_Init();
	MX_RTC_Init();
	MX_UART5_Init();
	MX_USB_DEVICE_Init();
	MX_UART7_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	//Initial startup procedure
	HAL_Delay(6000);
	HAL_GPIO_WritePin(Relay_EN_GPIO_Port, Relay_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(4000);
	HAL_GPIO_WritePin(ODrive_NRST_GPIO_Port, ODrive_NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(ODrive_NRST_GPIO_Port, ODrive_NRST_Pin, GPIO_PIN_SET);

	ODriveLink_Init(&huart5);
	HAL_Delay(500); /* let USB CDC enumerate before first log */

	cdc_log("Preparing move to (%.2f, %.2f) mm...\r\n", (double) TARGET_X_MM,
			(double) TARGET_Y_MM);
	motion_err_t err = Motion_PrepareMove();
	if (err != MOTION_ERR_NONE) {
		cdc_log("Motion_PrepareMove failed: err=%d - aborting.\r\n", (int) err);
	} else {
		cdc_log("Profile built. Starting 100 Hz streaming.\r\n");
		HAL_TIM_Base_Start_IT(&htim6);
		s_move_active = true;
	}

//Alterntative for interrup **Not checked with hardware
//  odrive_uart_init(&odrv, &huart5);

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
//  HAL_Delay(2000);
//  sprintf((char*)buffer,"Starting!");
//  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//
//
////  HAL_UART_Receive_IT(&huart5, uart_rx_buf, 1);
//  HAL_UART_Receive_IT(&huart5,uart5_rx_buf,1);
////
//  HAL_StatusTypeDef uart_status = HAL_UART_Transmit(&huart5, (uint8_t*)fullCallib, strlen(fullCallib), 1000);
//  HAL_Delay(10000);
//  HAL_UART_Transmit(&huart5, (uint8_t*)saveConfig, strlen(saveConfig), 1000);
//  HAL_Delay(5000);
	// Confirm the link is alive at all: force page 0
//  HAL_UART_Receive_IT(&huart4, &nx_rx_byte, 1); /* arm first byte */
//  nx_send(&huart4, "page 0");
//      HAL_Delay(100);

//  Packet_Init();

//	scara_app_init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (g_control_tick_flag) {
			g_control_tick_flag = 0;
			if (s_move_active) {
				bool still_running = Motion_StreamTick();
				if (!still_running) {
					s_move_active = false;
					HAL_TIM_Base_Stop_IT(&htim6);
					cdc_log("Move complete.\r\n");
					HAL_Delay(5000);
					HAL_GPIO_WritePin(MCU_Pneu_5_2_GPIO_Port, MCU_Pneu_5_2_Pin,
							GPIO_PIN_SET);
					HAL_Delay(2000);
					HAL_GPIO_WritePin(MCU_Pneu_3_GPIO_Port, MCU_Pneu_3_Pin,
							GPIO_PIN_SET);
					HAL_Delay(1000);
					HAL_GPIO_WritePin(MCU_Pneu_5_2_GPIO_Port, MCU_Pneu_5_2_Pin,
							GPIO_PIN_RESET);
					HAL_Delay(5000);
					HAL_GPIO_WritePin(MCU_Pneu_3_GPIO_Port, MCU_Pneu_3_Pin,
							GPIO_PIN_RESET);
					HAL_Delay(10000);
				}
			}
		}

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
//--closed loop motor control with FDCAN
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
//------------------Odrive testing with UART -----------------------------
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
//-------------Reading bus voltage from Odrive
//	  HAL_UART_Transmit(&huart5, (uint8_t*)read_voltage, strlen(read_voltage), 1000);
//	  HAL_Delay(4000);
//
//
//
//	  float vbus;
//	  if (odrive_read_property_f(&odrv, "vbus_voltage", &vbus) == HAL_OK) {
//	      sprintf((char*)buffer, "ODrive vbus = %.2f V\r\n", vbus);
//	  } else {
//	      sprintf((char*)buffer, "ODrive UART read failed (check wiring/baud)\r\n");
//	  }
//	  CDC_Transmit_FS(buffer, strlen((char*)buffer));
//	  HAL_UART_Transmit(&huart5, (uint8_t*)closedLoop0, strlen(closedLoop0), 1000);
//	  HAL_UART_Transmit(&huart5, (uint8_t*)closedLoop1, strlen(closedLoop1), 1000);
//--------------------------------------pNEUMATIC activation---------------------------------------
//	  HAL_GPIO_WritePin(MCU_Pneu_5_2_GPIO_Port, MCU_Pneu_5_2_Pin,GPIO_PIN_SET);
//	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(MCU_Pneu_3_GPIO_Port, MCU_Pneu_3_Pin, GPIO_PIN_SET);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(MCU_Pneu_5_2_GPIO_Port, MCU_Pneu_5_2_Pin,GPIO_PIN_RESET);
//	  HAL_Delay(2000);
//	  HAL_GPIO_WritePin(MCU_Pneu_3_GPIO_Port, MCU_Pneu_3_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(10000);
		//non-blocking, call every loop iteration
//		scara_app_poll();
//-----------Display test
//	  nx_send(&huart4, "t0.txt=\"GPIO HIGH\"");
//	         HAL_Delay(500);
//	         nx_send(&huart4, "t0.txt=\"GPIO LOW\"");
//	         HAL_Delay(500);
//------------Vision controller protocol-----------------
//	  if ((HAL_GetTick() - lastSendTick) >= SEND_INTERVAL_MS)
//	      {
//	          lastSendTick = HAL_GetTick();
//
//	          /* Replace with real data, e.g. current five-bar end-effector
//	           * coordinates from your kinematics module */
//	          sampleX += 10;
//	          sampleY += 5;
//
//	          uint8_t status = Packet_SendData(sampleX, sampleY, HAL_GetTick());
//
//	          if (status != USBD_OK)
//	          {
//	              char msg[64];
//	              sprintf(msg, "[TX FAIL] CDC busy or not connected (status=%d)\r\n", status);
//	              CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
//	          }
//
//	      }
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5
			| RCC_PERIPHCLK_UART7;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 12;
	PeriphClkInitStruct.PLL2.PLL2P = 2;
	PeriphClkInitStruct.PLL2.PLL2Q = 3;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.Usart234578ClockSelection =
	RCC_USART234578CLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//---------------------Interrupt callbak v1--------------------------------

	if (huart->Instance == UART5) {
		// Byte is already in uart_rx_buf[uart_rx_index] (HAL wrote it there)
		uart5_rx_index++;

		if (uart5_rx_index >= sizeof(uart5_rx_buf)) {
			CDC_Transmit_FS(uart5_rx_buf, uart5_rx_index);
			uart5_rx_index = 0;
		}

		// Re-arm for the NEXT byte, every single time
		HAL_UART_Receive_IT(&huart5, &uart5_rx_buf[uart5_rx_index], 1);
	}

//------------------Interrupt callback v2-----------------------
//	if(huart->Instance == ODRIVE_UART_HANDLE.Instance){
//		odrive_uart_rx_byte_isr(&g_odrive_ctx, g_odrive_ctx.rx_isr_byte);
//		HAL_UART_Receive_IT(huart,&g_odrive_ctx.rx_isr_byte,1);
//	}else if(huart->Instance == DISPLAY_UART_HANDLE.Instance){
//		odrive_uart_rx_byte_isr(&g_nextion_ctx, g_nextion_ctx.rx_isr_byte);
//		HAL_UART_Receive_IT(huart,&g_nextion_ctx.rx_isr_byte,1);
//	}

//---------------------Interrupt callback v3 for display testing

//	    if (huart->Instance == UART4) /* <- your Nextion UART instance */
//	    {
//	        uint8_t b = nx_rx_byte;
//
//	        if (nx_pkt_idx == 0 && b != 0x65) {
//	            /* not the start of a touch packet - ignore stray byte and
//	             * keep waiting for 0x65 */
//	        } else {
//	            nx_pkt[nx_pkt_idx++] = b;
//
//	            if (nx_pkt_idx == 7) {
//	                /* full packet assembled: validate the 0xFF 0xFF 0xFF tail */
//	                if (nx_pkt[4] == 0xFF && nx_pkt[5] == 0xFF && nx_pkt[6] == 0xFF) {
//	                    uint8_t component_id = nx_pkt[2];
//	                    uint8_t event = nx_pkt[3];
//
//	                    if (event == 0x00) { /* release only */
//	                        if (component_id == 0) {        /* b0 = LED ON */
//	                            HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);
//	                        } else if (component_id == 1) { /* b1 = LED OFF */
//	                            HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);
//	                        }
//	                    }
//	                }
//	                nx_pkt_idx = 0; /* ready for next packet */
//	            }
//	        }
//
//	        HAL_UART_Receive_IT(huart, &nx_rx_byte, 1); /* re-arm, always */
//	    }

}

uint32_t ODrive_Get_CAN_ID(uint8_t axis_id, uint32_t cmd_id) {
	return ((uint32_t) axis_id << 5) | cmd_id;
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

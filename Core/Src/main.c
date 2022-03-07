/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "canopen.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_RX_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef CanTxHeader;
uint8_t CanTxData[8] = {0};
CAN_RxHeaderTypeDef Can1RxHeader[8];
uint8_t Can1RxData[8][8] = {0};
CAN_RxHeaderTypeDef Can2RxHeader[8];
uint8_t Can2RxData[8][8] = {0};
uint32_t              Tx1Mailbox;
uint32_t              Tx2Mailbox;
volatile uint8_t rc1=0,rc2=0,ri1=0,ri2=0,ro1=0,ro2=0;

uint32_t RxCan1_cnt =0,RxCan2_cnt =0;
uint32_t TxCan1_cnt =0,TxCan2_cnt =0;
char DebugBuf[1024];
uint16_t Debug_len =0;

volatile uint8_t Can1_tx_done =1;
volatile uint8_t Can2_tx_done =1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_WWDG_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static void MX_CAN1_slientMode_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_SILENT;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char StrFunctionCode[16][10] = {"NMT      \0","EMERGENCY\0","TIME     \0","FC_3     \0","DOORS    \0","FC_5     \0","FC_6     \0","FC_7     \0","PDO_OUT  \0","PDO_IN   \0","MPDO     \0","TSDO     \0","RSDO     \0","FC_D     \0","HEARTBEAT\0","LSS      \0"};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
	if(hcan->Instance == hcan1.Instance)
	{
		  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can1RxHeader[ri1], Can1RxData[ri1]) != HAL_OK)
		  {
			/* Reception Error */
			  Error_Handler();
		  }
		  else
		  {
#if 0
			  uint8_t functioncode = ((Can1RxHeader[ri1].StdId>>3) & 0xF0);
			  uint8_t type = Can1RxData[ri1][0];
				uint16_t index;// = *(WORD *)&rx[ro][3];		/* read object index					*/
				uint8_t subindex;// = rx[ro][5];				/* read object subindex					*/
			  switch (functioncode) {
//				case 0x80:
//				case 0x00:
//				case 0xA0:
				case 0xC0:   //RDSO
					switch (type & COMMAND_SPECIFIER) {
						case INIT_WRITE_REQ:
							index = *(uint16_t *)&Can1RxData[ri1][1];		/* read object index					*/
							subindex = Can1RxData[ri1][3];				/* read object subindex					*/
							if (type & EXPEDITED_BIT)			/* expedited transfer					*/
							{
								switch (index) {
									case DOORRELAYTIME_CL:
//										  Debug_len =  sprintf(DebugBuf,"<= %s (%06lu)  0x%02lX 0x%02lX: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \r\n",StrFunctionCode[(functioncode>>4)],HAL_GetTick(),(Can1RxHeader[ri1].StdId &0x7F),((Can1RxHeader[ri1].StdId>>3) & 0xF0),Can1RxData[ri1][0],Can1RxData[ri1][1],Can1RxData[ri1][2],Can1RxData[ri1][3],Can1RxData[ri1][4],Can1RxData[ri1][5],Can1RxData[ri1][6],Can1RxData[ri1][7]);
//										  CDC_Transmit_FS((uint8_t *)DebugBuf, Debug_len);
										break;
									default:
										break;
								}

							}
							break;
						case (INIT_READ_REQ):					/* init read or expedited read			*/
							index = *(uint16_t *)&Can1RxData[ri1][1];		/* read object index					*/
							subindex = Can1RxData[ri1][3];				/* read object subindex					*/
							switch (index)
							{
								case LOAD_VALUE:
//									  Debug_len =  sprintf(DebugBuf,"<= %s (%06lu)  0x%02lX 0x%02lX: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \r\n",StrFunctionCode[(functioncode>>4)],HAL_GetTick(),(Can1RxHeader[ri1].StdId &0x7F),((Can1RxHeader[ri1].StdId>>3) & 0xF0),Can1RxData[ri1][0],Can1RxData[ri1][1],Can1RxData[ri1][2],Can1RxData[ri1][3],Can1RxData[ri1][4],Can1RxData[ri1][5],Can1RxData[ri1][6],Can1RxData[ri1][7]);
//									  CDC_Transmit_FS((uint8_t *)DebugBuf, Debug_len);
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}
					break;
					 case DOORS:
//									  Debug_len =  sprintf(DebugBuf,"<= %s (%06lu)  0x%02lX 0x%02lX: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \r\n",StrFunctionCode[(functioncode>>4)],HAL_GetTick(),(Can1RxHeader[ri1].StdId &0x7F),((Can1RxHeader[ri1].StdId>>3) & 0xF0),Can1RxData[ri1][0],Can1RxData[ri1][1],Can1RxData[ri1][2],Can1RxData[ri1][3],Can1RxData[ri1][4],Can1RxData[ri1][5],Can1RxData[ri1][6],Can1RxData[ri1][7]);
//									  CDC_Transmit_FS((uint8_t *)DebugBuf, Debug_len);
					  break;
				default:
					break;
			  }
#endif
			  RxCan1_cnt ++;
			  rc1++;
			  if(ri1 == (CAN_RX_SIZE -1)) { ri1 =0;}
			  else ri1++;

		  }
	}
	else	if (hcan->Instance == hcan2.Instance)
	{
		  if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Can2RxHeader[ri2], Can2RxData[ri2]) != HAL_OK)
		  {
			/* Reception Error */
			  Error_Handler();
		  }
		  else
		  {
//			  Debug_len =  sprintf(DebugBuf,"=> %s (%06lu)  0x%02lX 0x%02lX: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \r\n",StrFunctionCode[(Can2RxHeader[ri2].StdId>>7)&0x0F],HAL_GetTick(),(Can2RxHeader[ri2].StdId &0x7F),((Can2RxHeader[ri2].StdId>>3) & 0xF0),Can2RxData[ri2][0],Can2RxData[ri2][1],Can2RxData[ri2][2],Can2RxData[ri2][3],Can2RxData[ri2][4],Can2RxData[ri2][5],Can2RxData[ri2][6],Can2RxData[ri2][7]);
//			  CDC_Transmit_FS((uint8_t *)DebugBuf, Debug_len);
			  HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
			  RxCan2_cnt ++;
			  rc2++;
			  if(ri2 == (CAN_RX_SIZE -1)) { ri2 =0;}
			  else ri2++;

		  }
	}
}


int CNT_interrup[6] = {0};
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == hcan1.Instance)
	{
		TxCan1_cnt++;
		Can1_tx_done = 1;
		CNT_interrup[0]++;
	}
	else if(hcan->Instance == hcan2.Instance)
	{
		HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
		Can2_tx_done = 1;
		TxCan2_cnt++;
		CNT_interrup[1]++;
	}
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == hcan1.Instance)
	{
		TxCan1_cnt++;
		Can1_tx_done = 1;
		CNT_interrup[2]++;
	}
	else if(hcan->Instance == hcan2.Instance)
	{
		HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
		Can2_tx_done = 1;
		TxCan2_cnt++;
		CNT_interrup[3]++;
	}
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == hcan1.Instance)
	{
		TxCan1_cnt++;
		Can1_tx_done = 1;
		CNT_interrup[4]++;
	}
	else if(hcan->Instance == hcan2.Instance)
	{
		HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
		Can2_tx_done = 1;
		TxCan2_cnt++;
		CNT_interrup[5]++;
	}

}
int CAN1Mode =0;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
 // MX_WWDG_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  CAN1Mode = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
  if(CAN1Mode == 0)
  {
	  HAL_CAN_DeInit(&hcan1);
	  MX_CAN1_slientMode_Init();
  }
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x00;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
		Error_Handler();
	}
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterBank = 15;
	if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	  /* Start Error */
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	/* Notification Error */
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
	/* Notification Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
	  /* Start Error */
	  Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	/* Notification Error */
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
	/* Notification Error */
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t DebugTime =0;
  while (1)
  {
	  HAL_IWDG_Refresh(&hiwdg);

	  if(rc1)
	  {
		  CanTxHeader.StdId = Can1RxHeader[ro1].StdId;
		  CanTxHeader.RTR = Can1RxHeader[ro1].RTR;
		  CanTxHeader.IDE = Can1RxHeader[ro1].IDE;
		  CanTxHeader.DLC = Can1RxHeader[ro1].DLC;
		  CanTxHeader.TransmitGlobalTime = DISABLE;
		  memcpy(CanTxData,Can1RxData[ro1],8);
		  if(HAL_CAN_AddTxMessage(&hcan2, &CanTxHeader, CanTxData, &Tx1Mailbox) != HAL_OK)
		  {
			/* Transmission request Error */
			//Error_Handler();
		  }
		  else
		  {
			  Can2_tx_done =0;
			  if(ro1 == (CAN_RX_SIZE -1)) { ro1 =0;}
			  else ro1++;
			  rc1--;
		  }
	  }
	  if(rc2 )
	  {
		  CanTxHeader.StdId = Can2RxHeader[ro2].StdId;
		  CanTxHeader.RTR = Can2RxHeader[ro2].RTR;
		  CanTxHeader.IDE = Can2RxHeader[ro2].IDE;
		  CanTxHeader.DLC = Can2RxHeader[ro2].DLC;
		  CanTxHeader.TransmitGlobalTime = DISABLE;
		  memcpy(CanTxData,Can2RxData[ro2],8);
		  if(HAL_CAN_AddTxMessage(&hcan1, &CanTxHeader, CanTxData, &Tx2Mailbox) != HAL_OK)
		  {
			/* Transmission request Error */
			//Error_Handler();
		  }
		  else
		  {
			  Can1_tx_done =0;
			  if(ro2 == (CAN_RX_SIZE -1)) { ro2 =0;}
			  else ro2++;
			  rc2--;
		  }
	  }
//	  if(HAL_GetTick() > DebugTime)
//	  {
//		  DebugTime = HAL_GetTick() +500;
////		  HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
////		  Debug_len =  sprintf(DebugBuf,"(%lu) %lu %lu %lu %lu\n",HAL_GetTick(),RxCan1_cnt,TxCan2_cnt,RxCan2_cnt,TxCan1_cnt);
////		  CDC_Transmit_FS((uint8_t *)DebugBuf, Debug_len);
////		  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
////		  CanTxHeader.StdId = 0x01;
////		  CanTxHeader.RTR = CAN_RTR_DATA;
////		  CanTxHeader.IDE = CAN_ID_STD;
////		  CanTxHeader.DLC = 7;
////		  CanTxHeader.TransmitGlobalTime = DISABLE;
////		  if(HAL_CAN_AddTxMessage(&hcan1, &CanTxHeader, Can2RxData[ro1], &Tx1Mailbox) != HAL_OK)
////		  {
////			/* Transmission request Error */
////			Error_Handler();
////		  }
////		  else
////		  {
////		  }
//	  }
//	  HAL_IWDG_Refresh(&hiwdg);
//	  HAL_WWDG_Refresh(&hwwdg);706481665
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 18;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
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
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_RUN_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RUN_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LED_RUN_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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


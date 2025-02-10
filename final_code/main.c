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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


#define UPPER    85
#define DIPPER   68
#define STRAIGHT 83
#define LEFT     76
#define RIGHT    114
#define OFFSET   16
#define SENDER   0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/*    CAN PART     */
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

typedef struct data
{
	uint16_t    LDR;
	uint8_t     SMA;
	uint8_t     SOL;
	uint8_t     IND;
}sensorStruct;


sensorStruct TxSensorData;
sensorStruct RxSensorData;
sensorStruct read_struct;

/*    SERVO PART & LDR    */

uint8_t txbuffer[2];
uint8_t RX_x;
int8_t steer_tilt_x;
int8_t steer_angle;
uint16_t ldr;
uint8_t status ;
uint8_t ind;
uint16_t write_counter=0;
uint16_t write_packet_no=0;
uint16_t read_counter=0;
uint16_t read_packet_no=0;
uint8_t overflow_flag=0;
uint16_t high_byte,low_byte;

char data[20480];
uint8_t packet[5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

void CAN_filterConfig();
void SERVO();
void LDR();
void CAN_SENDER();
void servo_write(int);
void servo_sweep(void);
int map(int);
//void loadTxstruct();
//void loadRxstruct();

void data_logging_write(const sensorStruct );
void data_logging_read(char*);
void uart_func(const sensorStruct);
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* ACC_SERVO_MOTOR */
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
     HAL_GPIO_WritePin(GPIOE, SPI1_CS_Pin, 0); // CS Pulled Low
     txbuffer[0] = 0x20; // Address of Control Register 4
     txbuffer[1] = 0x31; // Data to be Filled
     HAL_SPI_Transmit(&hspi1, txbuffer, 2, 50);
     HAL_GPIO_WritePin(GPIOE, SPI1_CS_Pin, 1); // CS Pulled High

      /* CAN PART */

    /*##-Step1:Filter Configuration ###########################################*/
         CAN_filterConfig();

     /*##-Step2:Start the CAN peripheral ###########################################*/
         if (HAL_CAN_Start(&hcan1) != HAL_OK)
       	  {
       		/* Start Error */
       		Error_Handler();
       	  }
     /*##-Step3:Activate CAN RX notification #######################################*/
   	 if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   	{
   	  // Notification Error
   	  Error_Handler();
   	}

   	/*##-Step4:Configure Transmission process #####################################*/
   	  TxHeader.StdId = 0x123;
   	  TxHeader.RTR = CAN_RTR_DATA;
   	  TxHeader.IDE = CAN_ID_STD;
   	  TxHeader.DLC = 5;
   	  TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if SENDER
	  	SERVO();
	 	LDR();
	 	CAN_SENDER();
#endif

	 	 HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
  htim1.Init.Prescaler = 480;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RIGHT_Pin|DIPPER_Pin|LEFT_Pin|UPPER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_Pin DIPPER_Pin LEFT_Pin UPPER_Pin */
  GPIO_InitStruct.Pin = RIGHT_Pin|DIPPER_Pin|LEFT_Pin|UPPER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#if SENDER
void SERVO()
{
	// read the OUT_X register
		  	HAL_GPIO_WritePin(GPIOE, SPI1_CS_Pin, 0); // CS Pulled Low
		  	txbuffer[0] = 0x29 | 0x80; // read operation MSB = 1
		  	HAL_SPI_Transmit(&hspi1, txbuffer, 1, 50); // Transmit Data
		  	HAL_SPI_Receive(&hspi1, &RX_x, 1, 50); // Receive Data
		  	HAL_GPIO_WritePin(GPIOE, SPI1_CS_Pin, 1); // CS Pulled High
		  	servo_sweep();
}

void servo_sweep(void)
{
	if(RX_x>150)
	{
		steer_tilt_x = -(~RX_x+1);     //converting into original values
	}
	else
	{
		steer_tilt_x=RX_x;
	}

	steer_angle = steer_tilt_x * 1.3;
	if (steer_angle >= -5 && steer_angle <= 5)
	{
	    servo_write(90);
	    ind=STRAIGHT;
	    HAL_GPIO_WritePin(GPIOD,LEFT_Pin, RESET);
	    HAL_GPIO_WritePin(GPIOD,RIGHT_Pin, RESET);
	}
	else if (steer_angle > 5)
	{
		 servo_write(90 +  steer_angle);
			    ind=LEFT;
		HAL_GPIO_WritePin(GPIOD,LEFT_Pin, SET);
		HAL_GPIO_WritePin(GPIOD,RIGHT_Pin, RESET);
	}
	else if (steer_angle < -5)
	{
	    servo_write(-steer_angle);
	    	    ind=RIGHT;
	    HAL_GPIO_WritePin(GPIOD,LEFT_Pin, RESET);
	    HAL_GPIO_WritePin(GPIOD,RIGHT_Pin, SET);
	}

}

void servo_write(int angle)
{
	htim1.Instance->CCR1 = map(angle);
}

int map(int value)
{
    return (50+(value*200)/180);
}



void LDR()
{
	HAL_ADC_Start(&hadc1);
	ldr=HAL_ADC_GetValue(&hadc1);
	if(ldr>4000)
	{
		status=UPPER;
		HAL_GPIO_WritePin(GPIOD,UPPER_Pin,SET);
		HAL_GPIO_WritePin(GPIOD,DIPPER_Pin,RESET);

	}
	else if(ldr>150 && ldr<250)
	{
		status=DIPPER;
		HAL_GPIO_WritePin(GPIOD,UPPER_Pin,RESET);
		HAL_GPIO_WritePin(GPIOD,DIPPER_Pin,SET);
	}

	loadTxstruct();
}

void loadTxstruct()
{
	      TxSensorData.LDR=ldr;
	      TxSensorData.IND=ind;
	      TxSensorData.SMA=steer_angle;
	      TxSensorData.SOL=status ;
}




void CAN_SENDER()
{
	// Set the data to be transmitte expected
				TxData[0] = (((TxSensorData.LDR)& 0xFF00) >> 8) ;// upper nibble
				TxData[1] = (TxSensorData.LDR & 0x00FF);     //// lower nibble
				TxData[2] = TxSensorData.SMA;
				TxData[3] = TxSensorData.SOL;
				TxData[4] = TxSensorData.IND;

				// Start the Transmission process
				if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader,TxData, &TxMailbox) != HAL_OK)
				{
				// Transmission request Error
				Error_Handler();
				}

}
#else


void CAN_filterConfig(void)
{
	/*##- Setup the Filter to receive ANY MSG ID #####################################*/
	CAN_FilterTypeDef sFilterConfig;

	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}



// Get RX message from Fifo0 as message is Pending in Fifo0 to be Read
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	  // Get RX message from Fifo0 as message is Pending in Fifo to be Read
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    // Reception Error
    Error_Handler();
  }

  // Display LEDx
  if ((RxHeader.StdId == 0x123) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 5))
  {
    RxSensorData.LDR=((RxData[0] & 0x00FF) <<8) | (RxData[1]&0x00FF);
    RxSensorData.SMA=RxData[2];
    RxSensorData.SOL=RxData[3];
    RxSensorData.IND=RxData[4];

  data_logging_write(RxSensorData);
  }

}
void data_logging_write(const sensorStruct signals)
{
//writing in RAM

	write_counter=write_packet_no*OFFSET;

	data[write_counter]=(char)((signals.LDR & 0xFF00) >>8);//upper (char)((signals.LDR & 0xFF00) >>8)
	data[++write_counter]=(char)(signals.LDR & 0x00FF) ;//lower (char)((signals.LDR & 0x00FF))
	data[++write_counter]=(char)signals.SMA;
	data[++write_counter]=(char)signals.SOL;
	data[++write_counter]=(char)signals.IND;
	 write_packet_no++;

  if(write_packet_no>=1280)
  {
      		write_packet_no=0;   // function
      		overflow_flag=SET;   //for overflow condition
  }
  data_logging_read(data);
}

   void data_logging_read(char* data) // pointer to the buffer in which we are storing
   {


    if((read_packet_no < write_packet_no) || overflow_flag==SET) // in case 1278 (overflow flag)
    {
    read_counter=read_packet_no*OFFSET; //0*16    //FIFO

    high_byte =    ((data[read_counter] & 0x00FF) << 8);    // dividing into 8 bits
	read_counter++;
    low_byte =    data[read_counter]& 0x00FF;

    read_struct.LDR=     (high_byte | low_byte);			// combining to get original value
    read_counter++;
    read_struct.SMA=    (uint8_t)data[read_counter];
    read_counter++;
	read_struct.SOL=	(uint8_t)data[read_counter];
	read_counter++;
	read_struct.IND=    (uint8_t)data[read_counter];

    read_packet_no++;

        if(read_packet_no>=1280)
        {
      		read_packet_no=0; // function
        }

    }
		uart_func(read_struct);
		overflow_flag=RESET;

   }

  void uart_func(const sensorStruct signals)
   {
       		packet[0]=signals.SOL;
       		packet[1]=signals.SMA;
       		packet[2]=signals.IND;
       		packet[3]=(signals.LDR >>8);
       		packet[4]=(signals.LDR & 0xFF) ;

           status=HAL_UART_Transmit(&huart4,(uint8_t*)packet, sizeof(packet), 1000);
   }

#endif




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
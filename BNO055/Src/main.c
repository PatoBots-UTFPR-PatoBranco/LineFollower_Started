/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ini_BNO055(void);
void setup_BNO055(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// variaveis BNO055
uint8_t BufferI2C[50];

short int ax, ay, az; // aceleracoes
short int alx, aly, alz; // aceleracoeslineares
short int gx, gy, gz; // aceleracoes de gravidade
short int wx, wy, wz; // dados do giroscopio
short int mx, my, mz; // Dados do magnetometro
short int qw, qx, qy, qz; // dados dos quaternion
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // iniciar o BNO055 e o timer2 para captura dos dados
  setup_BNO055();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
uint8_t ini_BNO055(void)
{
	uint8_t IS_OK = 0;
	uint8_t Buff_aux[10]; //Leitura das ID dos elementos do Sensor
	if (HAL_I2C_IsDeviceReady(&hi2c1, BNO055_I2C_ADDR1 << 1, 1, 100)
			== HAL_OK) {
		//conseguiu conectar ao BNO
		//printDisplay("Conectou_BNO\n");
		//HAL_Delay(100);

		//mudar para a pagina 0 dos registradores
		Buff_aux[0] = BNO055_PAGE_ID_ADDR; //endereco
		Buff_aux[1] = BNO055_PAGE_ZERO; //dado
		HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, Buff_aux, 2,
				TIMEOUT);

		//Leitura dos ID do dispositivo
		Buff_aux[0] = BNO055_CHIP_ID_ADDR;
		HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, Buff_aux, 1,
				TIMEOUT);
		HAL_I2C_Master_Receive(&hi2c1, BNO055_I2C_ADDR1 << 1, &Buff_aux[1], 4,
				TIMEOUT);
		//BNO_ID= 0xA0 ACC_I=0xFB MAG_ID=0x32 GYRO_ID=0x0F
		if (Buff_aux[1] == 0xA0 && Buff_aux[2] == 0xFB && Buff_aux[3] == 0x32
				&& Buff_aux[4] == 0x0F) {
			IS_OK = 1;
		}

	}
	return IS_OK;
}

void setup_BNO055(void)
{
	while(ini_BNO055()!=1){}

	//configurar as condi��es de opera��o do BNO055
	//		  N�o precisa, por padr�o j� vem configurado dessa forma.
	//power mode ->Normal
	BufferI2C[0] = BNO055_PWR_MODE_ADDR;		 //endere�o
	BufferI2C[1] = BNO055_POWER_MODE_NORMAL;		 //dado
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	//Setar o modo de opera��o para configmode
	BufferI2C[0] = BNO055_OPR_MODE_ADDR;		 //endere�o
	BufferI2C[1] = BNO055_OPERATION_MODE_CONFIG;//dado do modo de opera��o
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	//Disparar o trigger do self test e setar o oscilador externo, e resetar BNO
	BufferI2C[0] = BNO055_SYS_TRIGGER_ADDR; //endere�o
	BufferI2C[1] = 0b11100001; //dado do modo de opera��o
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	HAL_Delay(700); // no datasheet diz que demora 650ms

	//		//Disparar o trigger do self test e setar o oscilador externo, e resetar BNO
	//		BufferI2C[0] = BNO055_SYS_TRIGGER_ADDR; //endere�o
	//		BufferI2C[1] = BNO055_CLK_SRC_MSK; //dado do modo de opera��o
	//		HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2,
	//		TIMEOUT);
	//
	//		HAL_Delay(700); // no datasheet diz que demora 650ms

	//Ler status do do selftest
	BufferI2C[0] = BNO055_SELFTEST_RESULT_ADDR; //Endere�o do status da calibra��o
	BufferI2C[1] = 0; // status inicial do self-test
	while ((BufferI2C[1] & 0xF) != 0xF) //enquanto n�o passar no teste, fica preso aqui
	{
		HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 1, TIMEOUT);
		HAL_I2C_Master_Receive(&hi2c1, BNO055_I2C_ADDR1 << 1, &BufferI2C[1], 1, TIMEOUT);

		HAL_Delay(100);
	}

	//mudar para a p�gina 1 de configura��o de registradores
	BufferI2C[0] = BNO055_PAGE_ID_ADDR; //endere�o
	BufferI2C[1] = BNO055_PAGE_ONE; //dado
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	//configura aceler�metro
	BufferI2C[0] = BNO055_ACCEL_CONFIG_ADDR; //endere�o
	BufferI2C[1] = 0b00000000; //000-normal_op_mode;000- 7Hz freq corte; 00 8G (Depois testar com 4G , 8G e 16G)
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	/*
	 // N�o adianta configurar no modo fus�o NDOF
	 //configura gyrosc�pio
	 BufferI2C[0]= BNO055_GYRO_CONFIG_ADDR;//endere�o
	 BufferI2C[1]= 0b00111011;//xx;111- 32Hz freq corte; 011 250 graus/se
	 HAL_I2C_Master_Transmit(&hi2c1,BNO055_I2C_ADDR1<<1,BufferI2C,2,TIMEOUT);

	 //configura magnet�metro
	 BufferI2C[0]= BNO055_MAG_CONFIG_ADDR;//endere�o
	 BufferI2C[1]= 0b00011101;//x;00- normal power mode; 11- Op mode high; 101 - 20Hz vel de transmis�o
	 HAL_I2C_Master_Transmit(&hi2c1,BNO055_I2C_ADDR1<<1,BufferI2C,2,TIMEOUT);
	 */

	//mudar para a p�gina 0 dos registradores
	BufferI2C[0] = BNO055_PAGE_ID_ADDR; //endere�o
	BufferI2C[1] = BNO055_PAGE_ZERO; //dado
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	//configura unidades de medida
	BufferI2C[0] = BNO055_UNIT_SEL_ADDR; //endere�o
	BufferI2C[1] = 0b00000110; //acel m/s^2; gyro rad/s; euler rad/s; Temp Celcius
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	//carrega a m�dia de 9 calibra��es  para ver se depois vai r�pido
	BufferI2C[0] = BNO055_ACCEL_OFFSET_X_LSB_ADDR; //endere�o
	BufferI2C[1] = 236; //ACC_OFFSET_X_LSB
	BufferI2C[2] = 255; //ACC_OFFSET_X_MSB
	BufferI2C[3] = 230; //ACC_OFFSET_Y_LSB
	BufferI2C[4] = 255; //ACC_OFFSET_Y_MSB
	BufferI2C[5] = 236; //ACC_OFFSET_Z_LSB
	BufferI2C[6] = 255; //ACC_OFFSET_Z_MSB
	BufferI2C[7] = 100; //MAG_OFFSET_X_LSB
	BufferI2C[8] = 0; //MAG_OFFSET_X_MSB
	BufferI2C[9] = 121; //MAG_OFFSET_Y_LSB
	BufferI2C[10] = 0; //MAG_OFFSET_Y_MSB
	BufferI2C[11] = 158; //MAG_OFFSET_Z_LSB
	BufferI2C[12] = 254; //MAG_OFFSET_Z_MSB
	BufferI2C[13] = 254; //GYR_OFFSET_X_LSB
	BufferI2C[14] = 255; //GYR_OFFSET_X_MSB
	BufferI2C[15] = 1; //GYR_OFFSET_Y_LSB
	BufferI2C[16] = 0; //GYR_OFFSET_Y_MSB
	BufferI2C[17] = 1; //GYR_OFFSET_Z_LSB
	BufferI2C[18] = 0; //GYR_OFFSET_Z_MSB
	BufferI2C[19] = 232; //ACC_RADIUS_LSB
	BufferI2C[20] = 3; //ACC_RADIUS_MSB
	BufferI2C[21] = 135; //MAG_RADIUS_LSB
	BufferI2C[22] = 2; //MAG_RADIUS_MSB
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 23, TIMEOUT);

	//		 setExtCrystalUse
	//		 set 0x80 (bit7) para usar o cristal externo
	//		 set modo de config
	BufferI2C[0] = BNO055_OPR_MODE_ADDR; //endere??o
	BufferI2C[1] = BNO055_OPERATION_MODE_CONFIG; //dado
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	HAL_Delay(25);

	//Disparar o trigger do self test e setar o oscilador externo, e resetar BNO
	BufferI2C[0] = BNO055_SYS_TRIGGER_ADDR; //endere�o
	BufferI2C[1] = BNO055_CLK_SRC_MSK; //dado do modo de opera��o
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	HAL_Delay(700); // no datasheet diz que demora 650ms

	//Setar o modo de opera��o
	BufferI2C[0] = BNO055_OPR_MODE_ADDR; //endere�o
	BufferI2C[1] = BNO055_OPERATION_MODE_NDOF; //dado do modo de opera��o
	//BufferI2C[1]= BNO055_OPERATION_MODE_NDOF_FMC_OFF;//dado do modo de opera��o
	//BufferI2C[1]= BNO055_OPERATION_MODE_AMG;//dado do modo de opera��o liga os 3 sensores
	HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 2, TIMEOUT);

	HAL_Delay(20); // ap�s mudar para um modo de op demora 7ms conforme o datasheeth

	//esperar o sensor terminar de calibrar par isso ficar mexendo o sensor em algumas posi��es diferentes
	BufferI2C[0] = BNO055_CALIB_STAT_ADDR; //Endere�o do status da calibra��o
	BufferI2C[1] = 0; // inicialmente n�o calibrado

	while (BufferI2C[1] != 0xFF) //enquanto n�o calibrar os tr�s sensores, fica preso aqui
	{
		HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 1, TIMEOUT);
		HAL_I2C_Master_Receive(&hi2c1, BNO055_I2C_ADDR1 << 1, &BufferI2C[1], 1, TIMEOUT);

		HAL_Delay(200);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		//Leitura simultanea dos 44 elementos na sequencia;
		BufferI2C[0] = BNO055_ACCEL_DATA_X_LSB_ADDR;

		HAL_I2C_Master_Transmit(&hi2c1, BNO055_I2C_ADDR1 << 1, BufferI2C, 1, TIMEOUT);
		HAL_I2C_Master_Receive(&hi2c1, BNO055_I2C_ADDR1 << 1, &BufferI2C[1], 44, TIMEOUT);

		ax = (BufferI2C[1] | BufferI2C[2] << 8);
		ay = (BufferI2C[3] | BufferI2C[4] << 8);
		az = (BufferI2C[5] | BufferI2C[6] << 8);
		mx = (BufferI2C[7] | BufferI2C[8] << 8);
		my = (BufferI2C[9] | BufferI2C[10] << 8);
		mz = (BufferI2C[11] | BufferI2C[12] << 8);
		wx = (BufferI2C[13] | BufferI2C[14] << 8);
		wy = (BufferI2C[15] | BufferI2C[16] << 8);
		wz = (BufferI2C[17] | BufferI2C[18] << 8);
		qw = (BufferI2C[25] | BufferI2C[26] << 8);
		qx = (BufferI2C[27] | BufferI2C[28] << 8);
		qy = (BufferI2C[29] | BufferI2C[30] << 8);
		qz = (BufferI2C[31] | BufferI2C[32] << 8);
		alx = (BufferI2C[33] | BufferI2C[34] << 8);
		aly = (BufferI2C[35] | BufferI2C[36] << 8);
		alz = (BufferI2C[37] | BufferI2C[38] << 8);
		gx = (BufferI2C[39] | BufferI2C[40] << 8);
		gy = (BufferI2C[41] | BufferI2C[42] << 8);
		gz = (BufferI2C[43] | BufferI2C[44] << 8);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

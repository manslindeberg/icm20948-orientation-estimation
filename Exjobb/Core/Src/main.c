/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "calc.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "ICM20948_SPI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TAMPERING_BUFFER_SIZE (20)
#define TAMPERING_UPPER_THRESHOLD (5.0)
#define TAMPERING_LOWER_THRESHOLD (0.3)

#define IMU_MOVABLE  (0)
#define IMU_FIXED   (1)

//#define TAMPERING_BUFFER 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
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
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_RTC_Init();
	MX_TIM16_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

	ICM_Initialize(&hspi1, &huart2, IMU_MOVABLE);
	ICM_Initialize(&hspi2, &huart2, IMU_FIXED);

	char uart_buffer[200];

	float accel_data_0[3] = {0,0,0};
	float accel_data_1[3] = {0,0,0};
	float gyro_data_0[3] = {0,0,0};
	float gyro_data_1[3] = {0,0,0};
	float accel_data_earthframe_0[3] = {0,0,0};
	float accel_data_earthframe_1[3] = {0,0,0};
	float gyro_bias_0[3] = {0,0,0};
	float gyro_bias_1[3] = {0,0,0};
	float accel_bias_0[2] = {0,0};
	float accel_bias_1[2] = {0,0};

	#ifdef TAMPERING_BUFFER
		float tampering_buffer_0[6][TAMPERING_BUFFER_SIZE];
		float tampering_buffer_1[6][TAMPERING_BUFFER_SIZE];
	#endif

	// High pass Filter Variables
	float low_pass_gyro_0[3] = {0,0,0};
	float low_pass_gyro_1[3] = {0,0,0};
	float prev_low_pass_gyro_0[3] = {0,0,0};
	float prev_low_pass_gyro_1[3] = {0,0,0};
	float low_alpha_gyro = 0.2;

	float low_pass_accel_0[3] = {0,0,0};
	float low_pass_accel_1[3] = {0,0,0};
	float prev_low_pass_accel_0[3] = {0,0,0};
	float prev_low_pass_accel_1[3] = {0,0,0};
	float low_alpha_acc = 0.2;

	struct quaternion quat_0 = {1,0,0,0};
	struct quaternion quat_1 = {1,0,0,0};
	#ifdef TAMPERING_BUFFER
		struct quaternion quat_buffer_0 = {1,0,0,0};
		struct quaternion quat_buffer_1 = {1,0,0,0};
	#endif
	struct euler_angles angles_0 = {0,0,0};
	struct euler_angles angles_1 = {0,0,0};
	struct euler_angles angles_buffer_0 = {0,0,0};
	struct euler_angles angles_buffer_1 = {0,0,0};
	struct euler_angles prev_0 = {0,0,0};
	struct euler_angles prev_1 = {0,0,0};
	struct euler_angles diff_0 = {0,0,0};
	struct euler_angles diff_1 = {0,0,0};
	struct matrix rotation_matrix_earth_0 = {0,0,0,0,0,0,0,0,0};
	struct matrix rotation_matrix_earth_1 = {0,0,0,0,0,0,0,0,0};

	float duration_diff = 0;
	float duration = 0;
	float clock = 16000000/16.0;
	float motion_duration = 0;

	float diff_pan_0;
	float diff_pan_1;
	float diff_tilt_0;
	float diff_tilt_1;
	float diff_pitch_0;
	float diff_pitch_1;
	float diff_pan_0_1;
	float diff_tilt_0_1;
	float diff_pitch_0_1;

	int8_t is_moving[3] = {0,0,0};
	int8_t was_moving = 0;
	int8_t moving_expected = 0;
	uint8_t uart_prescaler = 0;

	HAL_TIM_Base_Start(&htim16);
	ICM_AccCalibration(&hspi1,&huart2,accel_bias_0, IMU_MOVABLE);
	ICM_AccCalibration(&hspi2,&huart2,accel_bias_1, IMU_FIXED);
	CalculateRotationMatrix(accel_bias_0, &rotation_matrix_earth_0);
	CalculateRotationMatrix(accel_bias_1, &rotation_matrix_earth_1);
	ICM_GyroCalibration(&hspi1,&huart2, gyro_bias_0, IMU_MOVABLE);
	ICM_GyroCalibration(&hspi2,&huart2, gyro_bias_1, IMU_FIXED);

	sprintf(uart_buffer, "UART_PREAMBLE\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	__HAL_TIM_SET_COUNTER(&htim16,0);
	uart_prescaler = (uart_prescaler + 1) % 50;

	// IMU 0 READ DATA
	ICM_ReadGyroData(&hspi1, gyro_data_0, gyro_bias_0, IMU_MOVABLE);
	ICM_ReadAccData(&hspi1, accel_data_0, IMU_MOVABLE);

	// IMU 1 READ DATA
	ICM_ReadGyroData(&hspi2, gyro_data_1, gyro_bias_1, IMU_FIXED);
	ICM_ReadAccData(&hspi2, accel_data_1, IMU_FIXED);

	// Low-pass Filter Gyroscope & Acceleration IMU 0
	GyroLowPassFilter1(gyro_data_0, prev_low_pass_gyro_0, low_pass_gyro_0, low_alpha_gyro);
	GyroLowPassFilter1(accel_data_0, prev_low_pass_accel_0, low_pass_accel_0, low_alpha_acc);

	// Low-pass Filter Gyroscope & Acceleration IMU 0
	GyroLowPassFilter2(gyro_data_1, prev_low_pass_gyro_1, low_pass_gyro_1, low_alpha_gyro);
	GyroLowPassFilter2(accel_data_1, prev_low_pass_accel_1, low_pass_accel_1, low_alpha_acc);

	// Rotate Accelerometer from Earth frame into reference frame
	CalculateAccelerometerInEarthFrame(&rotation_matrix_earth_0, low_pass_accel_0, accel_data_earthframe_0);
	CalculateAccelerometerInEarthFrame(&rotation_matrix_earth_1, low_pass_accel_1, accel_data_earthframe_1);

	#ifdef TAMPERING_BUFFER
		// Record TAMPERING_BUFFER_SIZE of previous measurements
		for (uint8_t j = 0; j < 6; j++)
		{
			for (uint8_t i =  0; i < (TAMPERING_BUFFER_SIZE - 1); i++)
			{
				tampering_buffer_0[j][i+1] = tampering_buffer_0[j][i];
				tampering_buffer_1[j][i+1] = tampering_buffer_1[j][i];
			}
		}

		tampering_buffer_0[0][0] = accel_data_earthframe_0[0];
		tampering_buffer_0[1][0] = accel_data_earthframe_0[1];
		tampering_buffer_0[2][0] = accel_data_earthframe_0[2];
		tampering_buffer_0[3][0] = low_pass_gyro_0[0];
		tampering_buffer_0[4][0] = low_pass_gyro_0[1];
		tampering_buffer_0[5][0] = low_pass_gyro_0[2];

		tampering_buffer_1[0][0] = accel_data_earthframe_1[0];
		tampering_buffer_1[1][0] = accel_data_earthframe_1[1];
		tampering_buffer_1[2][0] = accel_data_earthframe_1[2];
		tampering_buffer_1[3][0] = low_pass_gyro_1[0];
		tampering_buffer_1[4][0] = low_pass_gyro_1[1];
		tampering_buffer_1[5][0] = low_pass_gyro_1[2];
	#endif

	// Detect angular movements
	if (low_pass_gyro_0[0] > TAMPERING_UPPER_THRESHOLD || low_pass_gyro_0[0] < -TAMPERING_UPPER_THRESHOLD)
	{
		is_moving[0] = 1;
	}

	if (low_pass_gyro_0[1] > TAMPERING_UPPER_THRESHOLD || low_pass_gyro_0[1] < -TAMPERING_UPPER_THRESHOLD)
	{
		is_moving[1] = 1;
	}

	if (low_pass_gyro_0[2] > TAMPERING_UPPER_THRESHOLD || low_pass_gyro_0[2] < -TAMPERING_UPPER_THRESHOLD)
	{
		is_moving[2] = 1;
	}

	if (low_pass_gyro_0[0] < TAMPERING_LOWER_THRESHOLD && low_pass_gyro_0[0] > -TAMPERING_LOWER_THRESHOLD)
	{
		is_moving[0] = 0;
	}

	if (low_pass_gyro_0[1] < TAMPERING_LOWER_THRESHOLD && low_pass_gyro_0[1] > -TAMPERING_LOWER_THRESHOLD)
	{
		is_moving[1] = 0;
	}
	if (low_pass_gyro_0[2] < TAMPERING_LOWER_THRESHOLD && low_pass_gyro_0[2] > -TAMPERING_LOWER_THRESHOLD)
	{
		is_moving[2] = 0;
	}

	// Camera went from not moving to moving
	if (is_moving[0] == 1 || is_moving[1] == 1 || is_moving[2] == 1)
	{
		MadgwickFilterXIO(low_pass_gyro_0, accel_data_earthframe_0, &quat_0);
		MadgwickFilterXIO(low_pass_gyro_1, accel_data_earthframe_1, &quat_1);
		was_moving = 1;

		motion_duration += SAMPLE_TIME_ICM;

		if (motion_duration > 10*1000)
		{
			// Convert Quaternion Space to Euler Angles Regular
			CalcQuaternionToEuler(quat_0, &angles_0);
			CalcQuaternionToEuler(quat_1, &angles_1);

			diff_pan_0 = (prev_0.yaw - angles_0.yaw);
			diff_pan_1 = (prev_1.yaw - angles_1.yaw);

			diff_tilt_0 = (prev_0.roll - angles_0.roll);
			diff_tilt_1 = (prev_1.roll - angles_1.roll);

			diff_pitch_0 = (prev_0.pitch - angles_0.pitch);
			diff_pitch_1 = (prev_1.pitch - angles_1.pitch);

			diff_pan_0_1 = diff_pan_0 - diff_pan_1;
			diff_tilt_0_1 = diff_tilt_0 - diff_tilt_1;
			diff_pitch_0_1 = diff_pitch_0 - diff_pitch_1;

			if (diff_pan_0_1 > 180.0)
			{
				diff_pan_0_1 = 360.0 - diff_pan_0_1;
			}

			if (diff_pan_0_1 < -180.0)
			{
				diff_pan_0_1 = -(diff_pan_0_1 + 360.0);
			}

			if (diff_tilt_0_1 > 180.0)
			{
				diff_tilt_0_1 = 360.0 - diff_tilt_0_1;
			}

			if (diff_tilt_0_1 < -180.0)
			{
				diff_tilt_0_1 = -(diff_tilt_0_1 + 360.0);
			}

			if (diff_pitch_0_1 > 180.0)
			{
				diff_pitch_0_1 = 360.0 - diff_pitch_0_1;
			}

			if (diff_pitch_0_1 < -180.0)
			{
				diff_pitch_0_1 = -(diff_pitch_0_1 + 360.0);
			}

			if ((fabs(diff_pan_0_1) > 3.0) | (fabs(diff_tilt_0_1) > 3.0) | (fabs(diff_pitch_0_1) > 3.0))
			{
				// Movement Expected - measured == expected?
				if (moving_expected)
				{
					sprintf(uart_buffer, "Movement Expected! measured: %.2f : %.2f : %.2f \r\n", diff_pan_0_1, diff_tilt_0_1, diff_pitch_0_1);
					HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				} else {
					// Movement NOT Expected - Tampering detected
					sprintf(uart_buffer, "Movement not Expected! tampering: %.2f : %.2f : %.2f \r\n", diff_pan_0_1, diff_tilt_0_1, diff_pitch_0_1);
					HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				}
			}

			// Update previous quaternion state
			prev_0.roll = angles_0.roll;
			prev_0.yaw = angles_0.yaw;
			prev_0.pitch = angles_0.pitch;
			prev_1.roll = angles_1.roll;
			prev_1.yaw = angles_1.yaw;
			prev_1.pitch = angles_1.pitch;
			motion_duration = 0;
		} else if (uart_prescaler == 0)
		{
			sprintf(uart_buffer, "MOVEMENT DURATION: %.3f \r\n", motion_duration);
			HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
		}
	} else {
		if (was_moving)
		{


			#ifdef TAMPERING_BUFFER
				// Integrate recording window samples before threshold IMU 0
				for (int8_t i = TAMPERING_BUFFER_SIZE-1; i >= 0; i--)
				{

					accel_data_0[0] = tampering_buffer_0[0][i];
					accel_data_0[1] = tampering_buffer_0[1][i];
					accel_data_0[2] = tampering_buffer_0[2][i];
					gyro_data_0[0] = tampering_buffer_0[3][i];
					gyro_data_0[1] = tampering_buffer_0[4][i];
					gyro_data_0[2] = tampering_buffer_0[5][i];
					MadgwickFilterXIO(gyro_data_0, accel_data_0, &quat_buffer_0);

				}

				// Integrate recording window samples before threshold IMU 0
				for (int8_t i = TAMPERING_BUFFER_SIZE-1; i >= 0; i--)
				{
					accel_data_1[0] = tampering_buffer_1[0][i];
					accel_data_1[1] = tampering_buffer_1[1][i];
					accel_data_1[2] = tampering_buffer_1[2][i];
					gyro_data_1[0] = tampering_buffer_1[3][i];
					gyro_data_1[1] = tampering_buffer_1[4][i];
					gyro_data_1[2] = tampering_buffer_1[5][i];
					MadgwickFilterXIO(gyro_data_1, accel_data_1, &quat_buffer_1);
				}

				// Convert Quaternion Space to Euler Angles Buffer
				CalcQuaternionToEuler(quat_buffer_0, &angles_buffer_0);
				CalcQuaternionToEuler(quat_buffer_1, &angles_buffer_1);
			#endif

			// Convert Quaternion Space to Euler Angles Regular
			CalcQuaternionToEuler(quat_0, &angles_0);
			CalcQuaternionToEuler(quat_1, &angles_1);



			#ifdef TAMPERING_BUFFER
				diff_pan_0 = (prev_0.yaw - angles_buffer_0.yaw) + (prev_0.yaw - angles_0.yaw);
				diff_pan_1 = (prev_1.yaw - angles_buffer_1.yaw) + (prev_1.yaw - angles_1.yaw);

				diff_tilt_0 = (prev_0.roll - angles_buffer_0.roll) + (prev_0.roll - angles_0.roll);
				diff_tilt_1 = (prev_1.roll - angles_buffer_1.roll) + (prev_1.roll - angles_1.roll);
				float temp1 = prev_0.yaw - angles_buffer_0.yaw;
				float temp2 = prev_0.yaw - angles_0.yaw;
				float temp3 = prev_1.yaw - angles_buffer_1.yaw;
				float temp4 = prev_1.yaw - angles_1.yaw;

				float temp5 = prev_0.roll - angles_buffer_0.roll;
				float temp6 = prev_0.roll - angles_0.roll;
				float temp7 = prev_1.roll - angles_buffer_1.roll;
				float temp8 = prev_1.roll - angles_1.roll;

				sprintf(uart_buffer, "diff_yaw_0 buffer: %.3f  other: %.3f \r\n", temp1, temp2);
				HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);

				sprintf(uart_buffer, "diff_yaw_1 buffer: %.3f  other: %.3f \r\n", temp3 ,temp4);
				HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);

				sprintf(uart_buffer, "diff_roll_0 buffer: %.3f  other: %.3f \r\n", temp5, temp6);
				HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);

				sprintf(uart_buffer, "diff_roll_1 buffer: %.3f  other: %.3f \r\n", temp7 ,temp8);
				HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
			#else
				diff_pan_0 = (prev_0.yaw - angles_0.yaw);
				diff_pan_1 = (prev_1.yaw - angles_1.yaw);

				diff_tilt_0 = (prev_0.roll - angles_0.roll);
				diff_tilt_1 = (prev_1.roll - angles_1.roll);

				diff_pitch_0 = (prev_0.pitch - angles_0.pitch);
				diff_pitch_1 = (prev_1.pitch - angles_1.pitch);
			#endif

			diff_pan_0_1 = diff_pan_0 - diff_pan_1;
			diff_tilt_0_1 = diff_tilt_0 - diff_tilt_1;
			diff_pitch_0_1 = diff_pitch_0 - diff_pitch_1;

			if (diff_pan_0_1 > 180.0)
			{
				diff_pan_0_1 = 360.0 - diff_pan_0_1;
			}

			if (diff_pan_0_1 < -180.0)
			{
				diff_pan_0_1 = -(diff_pan_0_1 + 360.0);
			}

			if (diff_tilt_0_1 > 180.0)
			{
				diff_tilt_0_1 = 360.0 - diff_tilt_0_1;
			}

			if (diff_tilt_0_1 < -180.0)
			{
				diff_tilt_0_1 = -(diff_tilt_0_1 + 360.0);
			}

			if (diff_pitch_0_1 > 180.0)
			{
				diff_pitch_0_1 = 360.0 - diff_pitch_0_1;
			}

			if (diff_pitch_0_1 < -180.0)
			{
				diff_pitch_0_1 = -(diff_pitch_0_1 + 360.0);
			}

			if ((fabs(diff_pan_0_1) > 3.0) | (fabs(diff_tilt_0_1) > 3.0) | (fabs(diff_pitch_0_1) > 3.0))
			{
				// Movement Expected - measured == expected?
				if (moving_expected)
				{
					sprintf(uart_buffer, "Movement Expected! measured: %.2f : %.2f : %.2f \r\n", diff_pan_0_1, diff_tilt_0_1, diff_pitch_0_1);
					HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				} else {
					// Movement NOT Expected - Tampering detected
					sprintf(uart_buffer, "Movement not Expected! tampering: %.2f : %.2f : %.2f \r\n", diff_pan_0_1, diff_tilt_0_1, diff_pitch_0_1);
					HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
				}
			}

			// Update previous quaternion state
			prev_0.roll = angles_0.roll;
			prev_0.yaw = angles_0.yaw;
			prev_0.pitch = angles_0.pitch;
			prev_1.roll = angles_1.roll;
			prev_1.yaw = angles_1.yaw;
			prev_1.pitch = angles_1.pitch;
			motion_duration = 0;
			// Update buffer to point at latest "non-moving" quaternion state
			#ifdef TAMPERING_BUFFER
				quat_buffer_0 = quat_0;
				quat_buffer_1 = quat_1;
			#endif
		}
		else if (uart_prescaler == 0)
		{
			sprintf(uart_buffer, "NO TAMPERING DETECTED -> MOVE EXPECTED: %i -> LOOP DURATION: %.3f \r\n", moving_expected, duration);
			HAL_UART_Transmit(&huart2,(uint8_t*) uart_buffer, strlen(uart_buffer), 1000);
		}
		was_moving = 0;
	}

	// Calculate loop execution duration and wait if not finished
	duration = (__HAL_TIM_GET_COUNTER(&htim16))*1000.0/clock;
	duration_diff = SAMPLE_TIME_ICM - duration;

	if(duration_diff > 0)
	{
	  HAL_Delay(duration_diff);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00303D5B;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 16;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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


/* MIT License

Copyright (c) 2022 Lindeberg, M & Hansson, L

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include "ICM20948_SPI.h"
#include "main.h"
#include "calc.h"
#include <string.h>
#include <stdio.h>

float magnetometer_offset[3] = {0,0,0}; // Hard Iron Correction
float magnetometer_scale[3] = {0,0,0}; // Soft Iron Correction
float magnet_offset = 0;	// Angle correction relative to "north"

/*
*
* SPI abstraction
* Creates a layer between STM32 HAL and the ICM library that will allow for easy platform swap
*
*/

float g_gyro_scale_factor;
float acc_scale_factor;
float offset_angles[2] = {0,0};


float mag_filter[3][MOVING_AVERAGE_SIZE];
int moving_average_size;

SPI_HandleTypeDef SPI;

/* Converting two uint8_t to int16_t */
#define UINT8_TO_INT16(dst, src_high, src_low) \
	do { \
	dst = (src_high); \
	dst <<= 8; \
	dst |= (src_low); \
} while (0);

void ICM_ReadBytes(SPI_HandleTypeDef* hspi, uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg | READ_FLAG;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &reg, 1,HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi, pData, Size,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void ICM_WriteBytes(SPI_HandleTypeDef* hspi, uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg & WRITE_FLAG;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &reg, 1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(hspi, pData, Size,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

}

void ICM_ReadOneByte(SPI_HandleTypeDef* hspi, uint8_t reg, uint8_t* pData) // ***
{
	reg = reg | READ_FLAG;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &reg, 1,HAL_MAX_DELAY);
	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Receive(hspi, pData, 1,HAL_MAX_DELAY);
	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void ICM_WriteOneByte(SPI_HandleTypeDef* hspi, uint8_t reg, uint8_t Data) // ***
{
	reg = reg & WRITE_FLAG;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &reg, 1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(hspi, &Data, 1,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void ICM_SelectBank(SPI_HandleTypeDef* hspi,uint8_t reg){
	ICM_WriteOneByte(hspi,USER_BANK_SEL,reg);
}


/*Initializing the ICM20602*/
void ICM_Initialize(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{

	uint8_t temp;

	ICM_SelectBank(hspi,USER_BANK_0);
	temp = PWR_MGT_RESET;
	ICM_WriteOneByte(hspi,REG_PWR_MGMT_1,temp);
	HAL_Delay(20);

	char uart_buffer[100];
	sprintf((char*) uart_buffer, "Initializing \r\n");
	HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);
	HAL_Delay(100);

	/* Who am i test
	 WhoAmI is in register bank 0* */
	ICM_SelectBank(hspi,USER_BANK_0);
	if (ICM_WHOAMI(hspi))
	{
		sprintf((char*) uart_buffer, "WHO AM I Success \r\n");
		HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);
	} else {
		sprintf((char*) uart_buffer, "WHO AM I Failed \r\n");
		HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);
	}

	HAL_Delay(50);

	uint8_t pwr_config = PWR_MGMT_1_CONFIG;
	uint8_t pwr_config2 = PWR_MGMT_2_CONFIG;

	ICM_WriteOneByte(hspi, REG_PWR_MGMT_1, pwr_config);
	HAL_Delay(20);
	ICM_WriteOneByte(hspi,REG_PWR_MGMT_2,pwr_config2);


	/*Turn Acceleration and Gyro OFF*/
	HAL_Delay(20);
	pwr_config = ACC_GYRO_OFF;
	ICM_WriteOneByte(hspi, REG_PWR_MGMT_2, pwr_config);

	/* Disable Fifo and I2C slave*/
	temp = 0b00010000;
	ICM_WriteOneByte(hspi, REG_USER_CTRL, temp);


	/* Configure Gyro */
	ICM_SelectBank(hspi,USER_BANK_2);
	temp = 0;
	ICM_WriteOneByte(hspi,REG_GYRO_CONFIG_2,temp);

	uint16_t dps = GYRO_DPS;
	if (ICM_GyroConfig(hspi, dps))
	{
		sprintf((char*) uart_buffer, "Gyro Configuration success DPS: %d \r\n", dps);
		HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);

	} else {
		sprintf((char*) uart_buffer, "Gyro Configuration failed \r\n");
		HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);

	}

		temp = 0;
		ICM_WriteOneByte(hspi,REG_ACCEL_CONFIG_2,temp);
	/*Turn Acceleartion and Gyro ON*/
		HAL_Delay(20);
		pwr_config = ACC_GYRO_ON;
		ICM_SelectBank(hspi,USER_BANK_0);
		HAL_Delay(20);
		ICM_WriteOneByte(hspi, REG_PWR_MGMT_2, pwr_config);
		HAL_Delay(20);
	/* Configure Accelerometer */
	uint8_t acc_scale = ACCEL_SCALE_SELECT;

	if(ICM_AccConfig(hspi,acc_scale)){
		sprintf((char*) uart_buffer,"Accelerometer Configuration success SELECT +-G: %d \r\n",acc_scale);
		HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);
	}else{
		sprintf((char*) uart_buffer, "Accelerometer Configuration failed \r\n");
		HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);
	}


	HAL_Delay(20);

	/*Configure*/


}


void ICM_CSHigh(void) {
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
}


void ICM_CSLow(void) {
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
}


/*Configuring Gyro DPS settings in Gyro Config Register */
uint8_t ICM_GyroConfig(SPI_HandleTypeDef *hspi, uint16_t dps)
{
	uint8_t config_byte;
	switch(dps)
	{
		case 250:
			config_byte = GYRO_250_DPS | GYRO_FILTER;
			g_gyro_scale_factor = GYRO_SCALE_250DPS;
			ICM_WriteOneByte(hspi, REG_GYRO_CONFIG_1, (uint8_t)config_byte);
			break;
		case 500:
			config_byte = GYRO_500_DPS | GYRO_FILTER;
			g_gyro_scale_factor = GYRO_SCALE_500DPS;
			ICM_WriteOneByte(hspi, REG_GYRO_CONFIG_1, (uint8_t)config_byte);
			break;
		case 1000:
			config_byte = GYRO_1000_DPS | GYRO_FILTER;
			g_gyro_scale_factor = GYRO_SCALE_1000DPS;
			ICM_WriteOneByte(hspi, REG_GYRO_CONFIG_1,(uint8_t)config_byte);
			break;
		case 2000:
			config_byte = GYRO_2000_DPS | GYRO_FILTER;
			g_gyro_scale_factor = GYRO_SCALE_2000DPS;
			ICM_WriteOneByte(hspi, REG_GYRO_CONFIG_1, (uint8_t)config_byte);
			break;
		default:
			config_byte = GYRO_1000_DPS | GYRO_FILTER;
			ICM_WriteOneByte(hspi, REG_GYRO_CONFIG_1, (uint8_t)config_byte);
			break;
	}

	HAL_Delay(10);

	uint8_t test = 0;

	ICM_ReadOneByte(hspi, REG_GYRO_CONFIG_1, &test);

	if (test != config_byte)
	{

		return 0;
	}
	return 1;
}


void ICM_GyroCalibration(SPI_HandleTypeDef *hspi,UART_HandleTypeDef* huart, float *gyro_bias)
{
	char uart_buffer[200];
	float gyro_data[3] = {0,0,0};
	float zero_bias[3] = {0,0,0};
	float gyro_accumulated[3] = {0,0,0};

	ICM_SelectBank(hspi,USER_BANK_0);
	HAL_Delay(10);

	for (int16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
	{
		ICM_ReadGyroData(hspi, gyro_data, zero_bias);
		gyro_accumulated[0] += gyro_data[0];
		gyro_accumulated[1] += gyro_data[1];
		gyro_accumulated[2] += gyro_data[2];
		HAL_Delay(20);
	}

	gyro_bias[0] =  -1*gyro_accumulated[0] / GYRO_CALIBRATION_SAMPLES;
	gyro_bias[1] =  -1*gyro_accumulated[1] / GYRO_CALIBRATION_SAMPLES;
	gyro_bias[2] =  -1*gyro_accumulated[2] / GYRO_CALIBRATION_SAMPLES;

	sprintf(uart_buffer,
					"\r\n Calibrating Gyroscope:"
					"(Gyro x-offset: %.5f | Gyro y-offset: %.5f | Gyro z-offset: %.5f)"
					"\r\n",
					gyro_bias[0], gyro_bias[1], gyro_bias[2]);
	HAL_UART_Transmit(huart, (uint8_t*) uart_buffer ,strlen(uart_buffer),1000);
}


/*
void ICM_AccCalibration(SPI_HandleTypeDef *hspi, UART_HandleTypeDef* huart, float *acc_bias)
{
	char uart_buffer[200];
	float acc_data[3] = {0,0,0};
	float zero_bias[3] = {0,0,0};
	float acc_accumulated[3] = {0,0,0};

	ICM_SelectBank(hspi,USER_BANK_0);
	HAL_Delay(10);

	for (int16_t i = 0; i < ACC_CALIBRATION_SAMPLES; i++)
	{
		ICM_ReadAccData(hspi, acc_data, zero_bias);
		acc_accumulated[0] += acc_data[0];
		acc_accumulated[1] += acc_data[1];
		acc_accumulated[2] += acc_data[2];
		HAL_Delay(20);
	}

	acc_bias[0] =  -1*acc_accumulated[0] / ACC_CALIBRATION_SAMPLES;
	acc_bias[1] =  -1*acc_accumulated[1] / ACC_CALIBRATION_SAMPLES;
	acc_bias[2] =  1 - (acc_accumulated[2] / ACC_CALIBRATION_SAMPLES);

	sprintf(uart_buffer,
					"\r\n Calibrating Accelerometer:"
					"(Acc x-offset: %.4f | Acc y-offset: %.4f | Acc z-offset: %.4f)"
					"\r\n",
					acc_bias[0], acc_bias[1], acc_bias[2]);
	HAL_UART_Transmit(huart, (uint8_t*)uart_buffer ,strlen(uart_buffer),1000);
}
**/

void ICM_AccCalibration(SPI_HandleTypeDef *hspi, UART_HandleTypeDef* huart, float *acc_bias){

	char uart_buffer[200];
	float acc_data[3] = {0,0,0};
	float acc_angle[2] = {0,0};
	struct euler_angles temp = {0,0,0};

	ICM_SelectBank(hspi,USER_BANK_0);
	for (int16_t i = 0; i < 500; i++)
	{
		ICM_ReadAccData(hspi, acc_data);
		CalcAccLinearToEuler(acc_data, &temp);
		acc_angle[0] += temp.roll;
		acc_angle[1] += temp.pitch;
		HAL_Delay(10);
	}

	float temp1 =  (acc_angle[0] / 500.0);
	float temp2 =  -(acc_angle[1] / 500.0);

	acc_bias[0] = temp1;
	acc_bias[1] = temp2;

	sprintf(uart_buffer,
		  "Accelerometer Calibration Succes: \r\n"
		  "Pitch: %.3f, Roll: %.3f",
		  temp1, temp2);
	HAL_UART_Transmit(huart,(uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}


void ICM_ReadGyroData(SPI_HandleTypeDef *hspi, float* gyro_data, float *gyro_bias)
{
	uint8_t gyro_raw[6] = {0,0,0,0,0,0};
	int16_t gyro_int[3] = {0,0,0};

	ICM_ReadBytes(hspi, REG_GYRO_XOUT_H, gyro_raw, 6);
	UINT8_TO_INT16(gyro_int[0], gyro_raw[0], gyro_raw[1]);
	UINT8_TO_INT16(gyro_int[1], gyro_raw[2], gyro_raw[3]);
	UINT8_TO_INT16(gyro_int[2], gyro_raw[4], gyro_raw[5]);

	gyro_data[0] = (((float) gyro_int[0]  / g_gyro_scale_factor) + gyro_bias[0]);
	gyro_data[1] = (((float) gyro_int[1]  / g_gyro_scale_factor) + gyro_bias[1]);
	gyro_data[2] = (((float) gyro_int[2]  / g_gyro_scale_factor) + gyro_bias[2]);
}


void ICM_ReadAccData(SPI_HandleTypeDef *hspi, float* accel_data){

	uint8_t acc_data[6] = {0,0,0,0,0,0};
	int16_t acc_int[3] = {0,0,0};
	ICM_ReadBytes(hspi,REG_ACCEL_XOUT_H,acc_data,6);

	UINT8_TO_INT16(acc_int[0],acc_data[0], acc_data[1]);
	UINT8_TO_INT16(acc_int[1],acc_data[2], acc_data[3]);
	UINT8_TO_INT16(acc_int[2],acc_data[4], acc_data[5]);

	accel_data[0] = (float)acc_int[0] / acc_scale_factor;
	accel_data[1] = (float)acc_int[1] / acc_scale_factor;
	accel_data[2] = (float)acc_int[2] / acc_scale_factor;
}

/*configure accelerometer sensitivity and scaler**/
uint8_t ICM_AccConfig(SPI_HandleTypeDef *hspi, uint8_t sensitivity){

	ICM_SelectBank(hspi,USER_BANK_2);
	uint8_t config_byte_acc;
	uint8_t config2 = 0;
	ICM_WriteOneByte(hspi,REG_ACCEL_CONFIG_2,(uint8_t)config2);

	switch(sensitivity){
	case 2:
		config_byte_acc = ACCEL_CONFIG_2G | ACC_FILTER;
		acc_scale_factor = ACCEL_SCALE_2G;
		ICM_WriteOneByte(hspi,REG_ACCEL_CONFIG,(uint8_t)config_byte_acc);
	break;
	case 4:
		config_byte_acc = ACCEL_CONFIG_4G | ACC_FILTER;
		acc_scale_factor = ACCEL_SCALE_4G;
		ICM_WriteOneByte(hspi,REG_ACCEL_CONFIG,(uint8_t)config_byte_acc);
	break;
	case 8:
		config_byte_acc = ACCEL_CONFIG_8G | ACC_FILTER;
		acc_scale_factor = ACCEL_SCALE_8G;
		ICM_WriteOneByte(hspi,REG_ACCEL_CONFIG,(uint8_t)config_byte_acc);
	break;
	case 16:
		config_byte_acc = ACCEL_CONFIG_16G | ACC_FILTER;
		acc_scale_factor = ACCEL_SCALE_16G;
		ICM_WriteOneByte(hspi,REG_ACCEL_CONFIG,(uint8_t)config_byte_acc);
	break;
	default:
		config_byte_acc = ACCEL_CONFIG_2G | ACC_FILTER;
		acc_scale_factor = ACCEL_SCALE_2G;
		ICM_WriteOneByte(hspi,REG_ACCEL_CONFIG,(uint8_t)config_byte_acc);
	}

	HAL_Delay(20);

	uint8_t test1 = 0;
	uint8_t test2 = 0;

	ICM_ReadOneByte(hspi,REG_ACCEL_CONFIG,&test1);
	ICM_ReadOneByte(hspi,REG_ACCEL_CONFIG_2,&test2);

	if((test1 != config_byte_acc) || (test2 != config2)){
		return 0;
	}else{
		return 1;
	}

}

uint8_t ICM_WHOAMI(SPI_HandleTypeDef *hspi) {
	uint8_t test = 0x00;
	ICM_ReadOneByte(hspi, REG_WHO_AM_I , &test);
	if (test != REG_WHO_AM_I_CONST)
	{
		return 0;
	} else {
		return 1;

	}
}


/*
 *
 * Read magnetometer applied with a moving average filter.
 *
 */

void ICM_ReadMagData(SPI_HandleTypeDef *hspi, float* mag_data, uint8_t filter, uint8_t hard_iron_comp, uint8_t soft_iron_comp) {
	uint8_t mag_buffer[6];
	int16_t mag_int[3];
	float mag_float[3];
	float mag_average[3] = {0,0,0};

	/*
	mag_buffer[0] = ICM_MagRead(hspi,0x11);
	mag_buffer[1] = ICM_MagRead(hspi,0x12);
	mag_buffer[2] = ICM_MagRead(hspi,0x13);
	mag_buffer[3] = ICM_MagRead(hspi,0x14);
	mag_buffer[4] = ICM_MagRead(hspi,0x15);
	mag_buffer[5] = ICM_MagRead(hspi,0x16);
	ICM_MagWrite(hspi,0x31, 0x01);
	 */

	ICM_MagReadMulti(hspi, 0x11, mag_buffer, 6);
	ICM_MagWrite(hspi,0x31, 0x01);
	UINT8_TO_INT16(mag_int[0], mag_buffer[1],mag_buffer[0]);
	UINT8_TO_INT16(mag_int[1], mag_buffer[3],mag_buffer[2]);
	UINT8_TO_INT16(mag_int[2], mag_buffer[5],mag_buffer[4]);

	mag_float[0] = ((float) mag_int[0] / MAG_SCALE_FACTOR);
	mag_float[1] = ((float) mag_int[1] / MAG_SCALE_FACTOR);
	mag_float[2] = ((float) mag_int[2] / MAG_SCALE_FACTOR);

	if (hard_iron_comp)
	{
		mag_float[0] -= magnetometer_offset[0];
		mag_float[1] -= magnetometer_offset[1];
		mag_float[2] -= magnetometer_offset[2];
	}

	if (soft_iron_comp)
	{
		mag_float[0] *= magnetometer_scale[0];
		mag_float[1] *= magnetometer_scale[1];
		mag_float[2] *= magnetometer_scale[2];
	}

	if (filter)
	{
		if (moving_average_size == (MOVING_AVERAGE_SIZE - 1))
		{
			for (uint8_t i = 0; i < (moving_average_size - 1); i++)
			{
				mag_filter[0][i] = mag_filter[0][i+1];
				mag_filter[1][i] = mag_filter[1][i+1];
				mag_filter[2][i] = mag_filter[2][i+1];
			}

			mag_filter[0][moving_average_size - 1] = mag_float[0];
			mag_filter[1][moving_average_size - 1] = mag_float[1];
			mag_filter[2][moving_average_size - 1] = mag_float[2];

		} else {
			mag_filter[0][moving_average_size] = mag_float[0];
			mag_filter[1][moving_average_size] = mag_float[1];
			mag_filter[2][moving_average_size] = mag_float[2];
			moving_average_size++;
		}

		for (uint8_t i = 0; i < moving_average_size; i++)
		{
			mag_average[0] += mag_filter[0][i];
			mag_average[1] += mag_filter[1][i];
			mag_average[2] += mag_filter[2][i];
		}

		mag_data[0] = (float) mag_average[0]/moving_average_size;
		mag_data[1] = (float) mag_average[1]/moving_average_size;
		mag_data[2] = (float) mag_average[2]/moving_average_size;

	} else {
		mag_data[0] = mag_float[0];
		mag_data[1] = mag_float[1];
		mag_data[2] = mag_float[2];
	}
	ICM_SelectBank(hspi,USER_BANK_0);
}


void ICM_MagConfig(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
	// Configure AUX_I2C Magnetometer (onboard ICM-20948)

	/*
	ICM_SelectBank(hspi, USER_BANK_0);
	//ICM_WriteOneByte(hspi,0x0F, 0x30);
	ICM_WriteOneByte(hspi,0x03, 0x20);
	ICM_SelectBank(hspi, USER_BANK_3);
	ICM_WriteOneByte(hspi,0x01, 0x4D);
	//ICM_WriteOneByte(hspi,0x02, 0x01); // Delay
	ICM_WriteOneByte(hspi,0x05, 0x81);
	ICM_MagWrite(hspi,0x32, 0x01);	// soft reset
	HAL_Delay(1000);
	ICM_MagWrite(hspi,0x31, 0x08);	// configure mode
	*/
	char uart_buffer[50];
	uint8_t new_val;

	// Enable I2C Master
	ICM_ReadOneByte(hspi, REG_USER_CTRL, &new_val);
	new_val |= 0x20;
	ICM_WriteOneByte(hspi, REG_USER_CTRL, new_val);

	// Reset
	ICM_ReadOneByte(hspi, AK09916_CNTL3, &new_val);
	new_val |= 0x01;
	ICM_MagWrite(hspi, AK09916_CNTL3, new_val);

	// Reset I2C Master
	ICM_SelectBank(hspi, USER_BANK_0);

	ICM_ReadOneByte(hspi, REG_USER_CTRL, &new_val);
	new_val |= 0x02;
	ICM_WriteOneByte(hspi, REG_USER_CTRL, new_val);

	// Enable I2C Master
	ICM_ReadOneByte(hspi, REG_USER_CTRL, &new_val);
	new_val |= 0x20;
	ICM_WriteOneByte(hspi, REG_USER_CTRL, new_val);

	HAL_Delay(100);

	ICM_SelectBank(hspi, USER_BANK_2);
	//ICM_WriteOneByte(hspi, ODR_REG_ALIGNMENT, 0x01); // Aligns ODR

	// Set Clock Frequency
	ICM_SelectBank(hspi, USER_BANK_3);
	ICM_ReadOneByte(hspi, I2C_MST_CTRL, &new_val);
	new_val |= 0x07;
	ICM_WriteOneByte(hspi, I2C_MST_CTRL, new_val);

	//ICM_ReadOneByte(hspi, 0x05, &new_val);
	//new_val |= 0x81;
	//ICM_WriteOneByte(hspi, 0x05, new_val);


	HAL_Delay(1000);
	// Set sampling 100Hz
	ICM_ReadOneByte(hspi, AK09916_CNTL2, &new_val);
	new_val |= 0x08;
	ICM_MagWrite(hspi, AK09916_CNTL2, 0x08);

	ICM_ReadOneByte(hspi, I2C_SLAVE_CTRL, &new_val);
	new_val |= 0x81;
	ICM_WriteOneByte(hspi, I2C_SLAVE_CTRL , new_val);

	// Control Who Am I
	uint8_t id = ICM_MagRead(hspi, 0x01);
	while (id != AK09916_ID)
	{
		id = ICM_MagRead(hspi, 0x01);
		sprintf((char*) uart_buffer, "AK09916 Who Am I Fail %x \r\n", id);
		HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);
		HAL_Delay(1000);
	}
	sprintf((char*) uart_buffer, "AK09916 Who Am I Success %x \r\n", id);
	HAL_UART_Transmit(huart, (uint8_t*) uart_buffer, strlen((char*) uart_buffer), 1000);
}



void ICM_MagWrite(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value) {
	ICM_SelectBank(hspi, USER_BANK_3);
	ICM_WriteOneByte(hspi, I2C_SLAVE_ADDR, MAG_WRITE | AK09916_ADDRESS);
	ICM_WriteOneByte(hspi, I2C_SLAVE_REG , reg);
	ICM_WriteOneByte(hspi, I2C_SLAVE_D0 , value);
	ICM_WriteOneByte(hspi, I2C_SLAVE_CTRL , 0x81);
}

uint8_t ICM_MagRead(SPI_HandleTypeDef *hspi, uint8_t reg) {
	uint8_t data;
	ICM_EnableMagRead(hspi, reg, 0x01);
	//HAL_Delay(2);
	ICM_SelectBank(hspi, USER_BANK_0);
	ICM_ReadOneByte(hspi, EXT_SLAVE_DATA, &data);
	ICM_EnableMagRead(hspi, AK09916_XOUT_L, 0x08);
	return data;
}

void ICM_MagReadMulti(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint8_t len) {
	ICM_EnableMagRead(hspi, reg, len);
	ICM_SelectBank(hspi, USER_BANK_0);
	//HAL_Delay(2);
	ICM_ReadBytes(hspi, EXT_SLAVE_DATA, data, len);
	ICM_EnableMagRead(hspi, AK09916_XOUT_L, 0x08);
}

void ICM_EnableMagRead(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t len)
{
	ICM_SelectBank(hspi, USER_BANK_3);
	ICM_WriteOneByte(hspi, I2C_SLAVE_ADDR, MAG_READ| AK09916_ADDRESS);
	ICM_WriteOneByte(hspi, I2C_SLAVE_REG , reg);
	ICM_WriteOneByte(hspi, I2C_SLAVE_CTRL , 0x80 | len);
	HAL_Delay(2);
}

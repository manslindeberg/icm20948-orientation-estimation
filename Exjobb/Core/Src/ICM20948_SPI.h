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

#include "main.h"
#include "calc.h"
#include <string.h>
#include <stdio.h>

#ifndef SRC_ICM20948_SPI_H_
#define SRC_ICM20948_SPI_H_

uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];
extern float magnet_offset;

#define ACC_CALIBRATION_SAMPLES (500)
#define GYRO_CALIBRATION_SAMPLES (500)

#define READ_FLAG 0x80
#define WRITE_FLAG 0x7F

#define MAG_READ 	(0x80)
#define MAG_WRITE	(0x00)

#define SAMPLE_TIME_ICM (20)
#define MOVING_AVERAGE_SIZE (25)

#define USER_BANK_SEL 0x7F
#define USER_BANK_0 (0b00000000)
#define USER_BANK_1 (0b00010000)
#define USER_BANK_2 (0b00100000)
#define USER_BANK_3 (0b00110000)

/*registers in user bank 0 **/
#define REG_WHO_AM_I 0x00
#define REG_USER_CTRL 0x03
#define REG_LP_CONFIG 0x05
#define REG_PWR_MGMT_1 0x06
#define REG_PWR_MGMT_2 0x07
#define REG_INT_PIN_CFG 0x0F
#define REG_INT_ENABLE 0x10
#define REG_INT_ENABLE_1 0x11
#define REG_INT_ENABLE_2 0x12
#define REG_INT_ENABLE_3 0x13
#define REG_ACCEL_XOUT_H 0x2D
#define REG_ACCEL_XOUT_L 0x2E
#define REG_ACCEL_YOUT_H 0x2F
#define REG_ACCEL_YOUT_L 0x30
#define REG_ACCEL_ZOUT_H 0x31
#define REG_ACCEL_ZOUT_L 0x32
#define REG_GYRO_XOUT_H 0x33
#define REG_GYRO_XOUT_L 0x34
#define REG_GYRO_YOUT_H 0x35
#define REG_GYRO_YOUT_L 0x36
#define REG_GYRO_ZOUT_H 0x37
#define REG_GYRO_ZOUT_L 0x38
#define REG_TEMP_OUT_H 0x39
#define REG_TEMP_OUT_L 0x3A

/*registers in user bank 1 **/
#define REG_SELF_TEST_X_GYRO 0x02
#define REG_SELF_TEST_Y_GYRO 0x03
#define REG_SELF_TEST_Z_GYRO 0x04
#define REG_SELF_TEST_X_ACCEL 0x0E
#define REG_SELF_TEST_Y_ACCEL 0x0F
#define REG_SELF_TEST_Z_ACCEL 0x10
#define REG_XA_OFFSET_H 0x14
#define REG_XA_OFFSET_L 0x15
#define REG_YA_OFFSET_H 0x17
#define REG_YA_OFFSET_L 0x18
#define REG_ZA_OFFSET_H 0x1A
#define REG_ZA_OFFSET_L 0x1B

/*registers in user bank 2 **/
#define REG_GYRO_CONFIG_1 0x01
#define REG_GYRO_CONFIG_2 0x02
#define REG_XG_OFFS_USRH 0x03
#define REG_XG_OFFS_USRL 0x04
#define REG_YG_OFFS_USRH 0x05
#define REG_YG_OFFS_USRL 0x06
#define REG_ZG_OFFS_USRH 0x07
#define REG_ZG_OFFS_USRL 0x08
#define ODR_REG_ALIGNMENT 0x09
#define REG_ACCEL_CONFIG 0x14
#define REG_ACCEL_CONFIG_2 0x15

/*registers in user bank 3 **/
#define I2C_MST_ODR_CONFIG		0x00 // Not found in MPU-9250
#define I2C_MST_CTRL       		0x01
#define I2C_MST_DELAY_CTRL 		0x02
#define I2C_SLV0_ADDR      		0x03
#define I2C_SLV0_REG       		0x04
#define I2C_SLV0_CTRL      		0x05
#define I2C_SLV0_DO        		0x06
#define I2C_SLV1_ADDR      		0x07
#define I2C_SLV1_REG       		0x08
#define I2C_SLV1_CTRL      		0x09
#define I2C_SLV1_DO        		0x0A
#define I2C_SLV2_ADDR      		0x0B
#define I2C_SLV2_REG       		0x0C
#define I2C_SLV2_CTRL      		0x0D
#define I2C_SLV2_DO        		0x0E
#define I2C_SLV3_ADDR      		0x0F
#define I2C_SLV3_REG       		0x10
#define I2C_SLV3_CTRL      		0x11
#define I2C_SLV3_DO        		0x12
#define I2C_SLV4_ADDR      		0x13
#define I2C_SLV4_REG       		0x14
#define I2C_SLV4_CTRL      		0x15
#define I2C_SLV4_DO        		0x16
#define I2C_SLV4_DI        		0x17

/*Magnetometer registers **/
#define AK09916_ADDRESS  0x0C
#define WHO_AM_I_AK09916 0x01 // (AKA WIA2) should return 0x09
#define AK09916_ST1		 0x10  // data ready status bit 0
#define AK09916_XOUT_L   0x11  // data
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL     0x30  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK09916_CNTL2    0x31  // Normal (0), Reset (1)
#define AK09916_ID		 0x09
#define AK09916_CNTL3 	 0x32

#define I2C_SLAVE_ADDR   0x03
#define I2C_SLAVE_REG	 0x04
#define I2C_SLAVE_CTRL	 0x05
#define I2C_SLAVE_D0	 0x06
#define EXT_SLAVE_DATA   0x3B


#define REG_SMPLRT_DIV 0x19
#define REG_LP_MODE_CFG 0x1E
#define REG_ACCEL_WOM_X_THR 0x20
#define REG_ACCEL_WOM_Y_THR 0x21
#define REG_ACCEL_WOM_Z_THR 0x22
#define REG_FIFO_EN 0x23
#define REG_FSYNC_INT 0x36
#define REG_FIFO_WM_INT_STATUS 0x39
#define REG_INT_STATUS 0x3A

#define REG_FIFO_WM_TH1 0x60
#define REG_FIFO_WM_TH2 0x62
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_ACCEL_INTEL_CTRL 0x69

#define REG_I2C_IF 0x70
#define REG_FIFO_COUNTH 0x72
#define REG_FIFO_COUNTL 0x73
#define REG_FIFO_R_W 0x74

#define REG_WHO_AM_I_CONST 0XEA
#define ICM_SPI_TIMEOUT 100000

/* Configured with filter (see page 59)*/
#define GYRO_250_DPS (0b00000000)
#define GYRO_500_DPS (0b00000010)
#define GYRO_1000_DPS (0b00000100)
#define GYRO_2000_DPS (0b00000110)
#define GYRO_DPS (1000)

#define GYRO_NO_FILTER (0b00000000)
#define GYRO_FILTER_1966 (0b00000001)
#define GYRO_FILTER_1518 (0b00001001)
#define GYRO_FILTER_1195 (0b00010001)
#define GYRO_FILTER_512 (0b00011001)
#define GYRO_FILTER_239 (0b00100001)
#define GYRO_FILTER_116 (0b00101001)
#define GYRO_FILTER_57 (0b00110001)
#define GYRO_FILTER_3614 (0b00111001)
#define GYRO_FILTER  (GYRO_FILTER_116)

#define GYRO_SCALE_250DPS 131.0
#define GYRO_SCALE_500DPS 65.5
#define GYRO_SCALE_1000DPS 32.8
#define GYRO_SCALE_2000DPS 16.4

/*Accelerometer configuration 1 see page 37 **/
#define ACCEL_CONFIG_2G (0b00000000)
#define ACCEL_CONFIG_4G (0b00000010)
#define ACCEL_CONFIG_8G (0b00110101)
#define ACCEL_CONFIG_16G (0b00000110)
#define ACCEL_SCALE_SELECT (8)

#define ACC_NO_FILTER (0b000000)
#define ACC_FILTER_246 (0b00000001)
#define ACC_FILTER_1114 (0b00010001)
#define ACC_FILTER_504 (0b00011001)
#define ACC_FILTER_239 (0b00100001)
#define ACC_FILTER_115 (0b00101001)
#define ACC_FILTER_57 (0b00110001)
#define ACC_FILTER_473 (0b00111001)
#define ACC_FILTER (ACC_FILTER_57)

#define ACCEL_SCALE_2G 16384.0
#define ACCEL_SCALE_4G 8192.0
#define ACCEL_SCALE_8G 4096.0
#define ACCEL_SCALE_16G 2048.0

#define PWR_MGT_RESET (0b10000000)
#define PWR_MGMT_1_CONFIG (0b00000001)
#define PWR_MGMT_2_CONFIG (0b00000000)
#define ACC_GYRO_ON (0b00000000)
#define ACC_GYRO_OFF (0b00111111)

#define MAG_SCALE_FACTOR (6.667752)

uint8_t ICM_WHOAMI(SPI_HandleTypeDef*);
uint8_t ICM_GyroConfig(SPI_HandleTypeDef*, uint16_t);
uint8_t ICM_AccConfig(SPI_HandleTypeDef*,uint8_t);
void ICM_Initialize(SPI_HandleTypeDef*, UART_HandleTypeDef*);
void ICM_ReadGyroData(SPI_HandleTypeDef*, float*, float*);
void ICM_ReadAccData(SPI_HandleTypeDef*,float*, float*);
void ICM_GyroCalibration(SPI_HandleTypeDef*,UART_HandleTypeDef*, float*);
void ICM_AccCalibration(SPI_HandleTypeDef*,UART_HandleTypeDef*, float*);
void ICM_SelectBank(SPI_HandleTypeDef*,uint8_t);
void ICM_ReadMagData(SPI_HandleTypeDef*, float*, uint8_t, uint8_t, uint8_t);
void ICM_MagPanCalibration(SPI_HandleTypeDef *hspi, UART_HandleTypeDef* huart);
uint8_t ICM_MagRead(SPI_HandleTypeDef*,uint8_t);
void ICM_MagWrite(SPI_HandleTypeDef*,uint8_t,uint8_t);
void ICM_MagConfig(SPI_HandleTypeDef *, UART_HandleTypeDef*);
void ICM_MagReadMulti(SPI_HandleTypeDef*, uint8_t, uint8_t*, uint8_t);
void ICM_EnableMagRead(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t len);

void ICM_CSHigh(void);
void ICM_CSLow(void);

#endif /* SRC_ICM20948_SPI_H_ */

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

#ifndef SRC_CALC_H_
#define SRC_CALC_H_
#include "math.h"

#define TAMPERING_UPPER_THRESHOLD (5.0)
#define TAMPERING_LOWER_THRESHOLD (0.3)

#define RAD_2_DEG (180.0/M_PI)
#define DEG_2_RAD (M_PI/180.0)

#define GYRO_THRESHOLD_250 (2*0.0076*DEG_2_RAD)
#define GYRO_THRESHOLD_500 (2*0.0153*DEG_2_RAD)
#define GYRO_THRESHOLD_1000 (10*0.0305*DEG_2_RAD)
#define GYRO_THRESHOLD_2000 (2*0.0608*DEG_2_RAD)
#define GYRO_THRESHOLD (GYRO_THRESHOLD_1000)


struct quaternion {
    float q1;
    float q2;
    float q3;
    float q4;
};

struct euler_angles {
	float roll;
	float pitch;
	float yaw;
};

struct cartesian_vector {
	float x;
	float y;
	float z;
};

struct matrix {
	float a11;
	float a12;
	float a13;
	float a21;
	float a22;
	float a23;
	float a31;
	float a32;
	float a33;
};

void CalcQuaternionToEuler(struct quaternion quat, struct euler_angles*);
void CalcQuaternionToEuler2(struct quaternion quat, struct euler_angles *eu);
void MahonyFilterXIO(float*, float*, struct quaternion*);
void MadgwickFilterXIO(float *, float*, struct quaternion *);
void MadgwickFilterArduino(float *, float*, struct quaternion *);
void CalcGyroQuaternion(float* gyro_data, struct quaternion*);
void CalcGyroEuler(float *gyro_data, struct euler_angles*);
void CalcAccLinearToEuler(float* accel_data, struct euler_angles*);
void GyroHighPassFilter(float*, float*, float*, float*, float);
void GyroLowPassFilter1(float *gyro_data, float* prev_filt, float* filt, float a);
void GyroLowPassFilter2(float *gyro_data, float* prev_filt, float* filt, float a);
void MahonyFilter(float*, float*, struct quaternion *);
void CalculateRotationMatrix(float*, struct matrix *);
void CalculateAccelerometerInEarthFrame(struct matrix *, float*, float*);
void CalcAngleDifference(struct euler_angles*, struct euler_angles*, struct euler_angles*, struct euler_angles*);
void MarkIsMoving(float *gyro_data, float *is_moving);
struct quaternion q_multiplication(struct quaternion quad_left, struct quaternion quad_right);
struct quaternion q_add(struct quaternion quad_left, struct quaternion quad_right);
struct quaternion q_subtract(struct quaternion quad_left, struct quaternion quad_right);
struct quaternion q_inverse(struct quaternion quad);

void q_normalize(struct quaternion * quad);
void q_scalar(struct quaternion *quad, float s);

#endif /* SRC_CALC_H_ */

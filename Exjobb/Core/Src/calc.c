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

#include <math.h>
#include "main.h"
#include "calc.h"
#include <string.h>
#include <stdio.h>
#include "ICM20948_SPI.h"

/* Mahony Filter Parameters */
static float k_i = 0.5;
static float k_p = 0.2;
float eInt[3] = {0,0,0};

/*Madgwick Filter Parameters */
static float Beta = 0.1;


/* Converts Quaternion coordinates into Euler Angles */
void CalcQuaternionToEuler(struct quaternion quat, struct euler_angles* eu)
{
	eu->roll = 90 - atan2f((quat.q1*quat.q2 + quat.q3*quat.q4), 0.5 - (quat.q2*quat.q2 + quat.q3*quat.q3))*RAD_2_DEG;
	eu->pitch = asinf(2.0*(quat.q1*quat.q3 - quat.q2*quat.q4))*RAD_2_DEG;
	eu->yaw = -atan2f((quat.q2*quat.q3 + quat.q1*quat.q4), 0.5 - (quat.q3*quat.q3 + quat.q4*quat.q4))*RAD_2_DEG; //clockwise direction
}


/* Simple Riemann-integration of gyroscope data */
void CalcGyroEuler(float *gyro_data, struct euler_angles* eu_gyro_est)
{
	eu_gyro_est->yaw += gyro_data[2]*SAMPLE_TIME_ICM/1000.0;
	eu_gyro_est->pitch += gyro_data[1]*SAMPLE_TIME_ICM/1000.0;
	eu_gyro_est->roll += gyro_data[0]*SAMPLE_TIME_ICM/1000.0;
}


/* Calculates Pitch & roll from direction of gravity */
void CalcAccLinearToEuler(float* accel_data, struct euler_angles* eu_acc_est)
{
	float pitch = atan(accel_data[0]/accel_data[2])* RAD_2_DEG;
	float roll = atan(accel_data[1]/sqrt(pow(accel_data[0],2) + pow(accel_data[2],2))) * RAD_2_DEG;

	eu_acc_est->roll = roll;
	eu_acc_est->pitch = pitch;
}


/* Quaternion integration using Runge-Kutta method. */
void CalcGyroQuaternion(float* gyro_data, struct quaternion *q)
{
	struct quaternion q_gyro_rate = {0, gyro_data[0], gyro_data[1], gyro_data[2]};

	q_gyro_rate = q_multiplication(*q, q_gyro_rate);
	q_scalar(&q_gyro_rate,0.5 * SAMPLE_TIME_ICM/1000.0);
	*q = q_add(q_gyro_rate, *q);
	q_normalize(q);
}


/* Credit to xio-technologies for open-source implementation https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU*/
void MahonyFilterXIO(float *gyro_data, float *accel_data, struct quaternion *q)
{
	float q1 = q->q1, q2 = q->q2, q3 = q->q3, q4 = q->q4;   // short name local variable for readability
	float norm;
	float ax = accel_data[0], ay = accel_data[1], az = accel_data[2];
	float gx = gyro_data[0]*DEG_2_RAD, gy = gyro_data[1]*DEG_2_RAD, gz = gyro_data[2]*DEG_2_RAD;
	float vx, vy, vz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Normalise accelerometer measurement
	norm = (float) sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0) return; // handle NaN
	norm = 1.0 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Estimated direction of gravity
	vx = 2.0 * (q2 * q4 - q1 * q3);
	vy = 2.0 * (q1 * q2 + q3 * q4);
	vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	if (k_i > 0.0)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0;     // prevent integral wind up
		eInt[1] = 0.0;
		eInt[2] = 0.0;
	}

	// Zero gyroscope measurements if they are below the threshold. Only if threshold is defind
	#ifdef GYRO_THRESHOLD
		if (gx < GYRO_THRESHOLD && gx > -GYRO_THRESHOLD)
		{
			gx = 0.0;
		}

		if (gy < GYRO_THRESHOLD && gy > -GYRO_THRESHOLD)
		{
			gy = 0.0;
		}

		if (gz < GYRO_THRESHOLD && gz > -GYRO_THRESHOLD)
		{
			gz = 0.0;
		}
	#endif

	// Apply feedback terms
	gx = gx + k_p * ex + k_i * eInt[0];
	gy = gy + k_p * ey + k_i * eInt[1];
	gz = gz + k_p * ez + k_i * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * SAMPLE_TIME_ICM/1000.0);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 * SAMPLE_TIME_ICM/1000.0);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 * SAMPLE_TIME_ICM/1000.0);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 * SAMPLE_TIME_ICM/1000.0);

	// Normalise quaternion
	norm = (float) sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0 / norm;
	q->q1 = q1 * norm;
	q->q2 = q2 * norm;
	q->q3 = q3 * norm;
	q->q4 = q4 * norm;
}


/* Credit to xio-technologies for open-source implementation https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU*/
void MadgwickFilterXIO(float *gyro_data, float *accel_data, struct quaternion *q)
{
	float q1 = q->q1, q2 = q->q2, q3 = q->q3, q4 = q->q4;   // short name local variable for readability
	float ax = accel_data[0], ay = accel_data[1], az = accel_data[2];
	float gx = gyro_data[0]*DEG_2_RAD, gy = gyro_data[1]*DEG_2_RAD, gz = gyro_data[2]*DEG_2_RAD;
	float norm;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1 = 2.0 * q1;
	float _2q2 = 2.0 * q2;
	float _2q3 = 2.0 * q3;
	float _2q4 = 2.0 * q4;
	float _4q1 = 4.0 * q1;
	float _4q2 = 4.0 * q2;
	float _4q3 = 4.0 * q3;
	float _8q2 = 8.0 * q2;
	float _8q3 = 8.0 * q3;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0) return; // handle NaN
	norm = 1.0 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Gradient decent algorithm corrective step
	s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
	s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
	s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
	s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay;
	norm = 1.0 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
	qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
	qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
	qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * SAMPLE_TIME_ICM/1000.0;
	q2 += qDot2 * SAMPLE_TIME_ICM/1000.0;
	q3 += qDot3 * SAMPLE_TIME_ICM/1000.0;
	q4 += qDot4 * SAMPLE_TIME_ICM/1000.0;

	norm = 1 / (float) sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	q->q1 = q1 * norm;
	q->q2 = q2 * norm;
	q->q3 = q3 * norm;
	q->q4 = q4 * norm;
}


void MadgwickFilterArduino(float *gyro_data, float *accel_data, struct quaternion *q)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float ax = accel_data[0], ay = accel_data[1], az = accel_data[2];
	float gx = gyro_data[0]*DEG_2_RAD, gy = gyro_data[1]*DEG_2_RAD, gz = gyro_data[2]*DEG_2_RAD;
	float q0 = q->q1;
	float q1 = q->q2;
	float q2 = q->q3;
	float q3 = q->q4;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1.0/sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = 1.0/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= Beta * s0;
		qDot2 -= Beta * s1;
		qDot3 -= Beta * s2;
		qDot4 -= Beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * SAMPLE_TIME_ICM/1000.0;
	q1 += qDot2 * SAMPLE_TIME_ICM/1000.0;
	q2 += qDot3 * SAMPLE_TIME_ICM/1000.0;
	q3 += qDot4 * SAMPLE_TIME_ICM/1000.0;

	// Normalise quaternion
	recipNorm = 1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	q->q1 = q0;
	q->q2 = q1;
	q->q3 = q2;
	q->q4 = q3;
}


/* Exponential Weighted Moving Average Low Pass Filter */
void GyroLowPassFilter(float *gyro_data, float* prev_filt, float* filt, float a)
{
	// Calculating new high-pass filtered data
	filt[0] = prev_filt[0] + a*(gyro_data[0] - prev_filt[0]);
	filt[1] = prev_filt[1] + a*(gyro_data[1] - prev_filt[1]);
	filt[2] = prev_filt[2] + a*(gyro_data[2] - prev_filt[2]);

	prev_filt[0] = filt[0];
	prev_filt[1] = filt[1];
	prev_filt[2] = filt[2];
}


/* Exponential Weighted Moving Average High Pass Filter */
void GyroHighPassFilter(float *gyro_data, float* prev_gyro_data, float* prev_filt, float* filt, float a)
{
	// Calculating new high-pass filtered data
	filt[0] = a*prev_filt[0] + a*(gyro_data[0] - prev_gyro_data[0]);
	filt[1] = a*prev_filt[1] + a*(gyro_data[1] - prev_gyro_data[1]);
	filt[2] = a*prev_filt[2] + a*(gyro_data[2] - prev_gyro_data[2]);

	// Update previous vectors
	prev_gyro_data[0] = gyro_data[0];
	prev_gyro_data[1] = gyro_data[1];
	prev_gyro_data[2] = gyro_data[2];

	prev_filt[0] = filt[0];
	prev_filt[1] = filt[1];
	prev_filt[2] = filt[2];
}


struct quaternion q_multiplication(struct quaternion quad_left, struct quaternion quad_right)
{
	struct quaternion result = {0,0,0,0};
    result.q1 = (quad_left.q1 * quad_right.q1) - (quad_left.q2 * quad_right.q2) - (quad_left.q3 * quad_right.q3) - (quad_left.q4 * quad_right.q4);
    result.q2 = (quad_left.q1 * quad_right.q2) + (quad_left.q2 * quad_right.q1) + (quad_left.q3 * quad_right.q4) - (quad_left.q4 * quad_right.q3);
    result.q3 = (quad_left.q1 * quad_right.q3) - (quad_left.q2 * quad_right.q4) + (quad_left.q3 * quad_right.q1) + (quad_left.q4 * quad_right.q2);
    result.q4 = (quad_left.q1 * quad_right.q4) + (quad_left.q2 * quad_right.q3) - (quad_left.q3 * quad_right.q2) + (quad_left.q4 * quad_right.q1);
    return result;
}


void q_scalar(struct quaternion *quad, float s)
{
	quad->q1 *= s;
	quad->q2 *= s;
	quad->q3 *= s;
	quad->q4 *= s;
}


struct quaternion q_add(struct quaternion quad_left, struct quaternion quad_right)
{
	struct quaternion result;
	result.q1 = quad_left.q1 + quad_right.q1;
	result.q2 = quad_left.q2 + quad_right.q2;
	result.q3 = quad_left.q3 + quad_right.q3;
	result.q4 = quad_left.q4 + quad_right.q4;

	return result;
}


struct quaternion q_subtract(struct quaternion quad_left, struct quaternion quad_right)
{
	struct quaternion result;
	result.q1 = quad_left.q1 - quad_right.q1;
	result.q2 = quad_left.q2 - quad_right.q2;
	result.q3 = quad_left.q3 - quad_right.q3;
	result.q4 = quad_left.q4 - quad_right.q4;

	return result;
}


struct quaternion q_inverse(struct quaternion quad){
	struct quaternion inverse = {0,0,0,0};
	inverse.q1 = quad.q1;
    inverse.q2 = -quad.q2;
    inverse.q3 = -quad.q3;
    inverse.q4 = -quad.q4;
    return inverse;
}


void q_normalize(struct quaternion * quad)
{
	float length =  sqrt(pow(quad->q1,2) + pow(quad->q2,2) + pow(quad->q3,2) + pow(quad->q4,2));

	quad->q1 /= length;
	quad->q2 /= length;
	quad->q3 /= length;
	quad->q4 /= length;

}

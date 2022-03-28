#include <math.h>
#include "main.h"
#include "calc.h"
#include <string.h>
#include <stdio.h>
#include "ICM20948_SPI.h"

/* Mahony Filter Parameters */
static float k_i = 1.1;
static float k_p = 0.4;

float i[3] = {0,0,0};

void CalcQuaternionToEuler(struct quaternion quat, struct euler_angles* eu)
{
	eu->roll = 90 - atan2((quat.q1*quat.q2 + quat.q3*quat.q4), 0.5 - (quat.q2*quat.q2 + quat.q3*quat.q3))*RAD_2_DEG;
	eu->pitch = asin(2.0*(quat.q1*quat.q3 - quat.q2*quat.q4))*RAD_2_DEG;
	eu->yaw = -atan2((quat.q2*quat.q3 + quat.q1*quat.q4), 0.5 - (quat.q3*quat.q3 + quat.q4*quat.q4))*RAD_2_DEG;
}

void CalcQuaternionToEuler2(struct quaternion quat, struct euler_angles *eu)
{
	float qmod = sqrt(pow(quat.q1,2) + pow(quat.q2,2) +
			pow(quat.q3,2) + pow(quat.q4,2));
	float q_t = quat.q1*quat.q3 - quat.q1*quat.q4;

	/* Checking for singularities */
	if(q_t > qmod/2)
	{
		eu->yaw = 2*atan2f(quat.q2, quat.q1)*RAD_2_DEG;
		eu->pitch = 90.0;
		eu->roll = 0.0;
	}else if(q_t < -qmod/2)
	{
		eu->yaw = -2*atan2f(quat.q2, quat.q1)*RAD_2_DEG;
		eu->pitch = -90.0;
		eu->roll = 0.0;
	}else{
		eu->yaw = atan2f((2*quat.q1*quat.q4 + quat.q2*quat.q3),
				pow(quat.q1,2) + pow(quat.q2,2) - pow(quat.q3,2) - pow(quat.q4,2))*RAD_2_DEG;
		if (2*q_t/qmod < 1 && 2*q_t/qmod >= -1)
		{
			eu->pitch = asin(2*q_t/qmod)*RAD_2_DEG;
		}
		eu->roll = atan2f((2*quat.q1*quat.q2 + quat.q3*quat.q4),
					pow(quat.q1,2) - pow(quat.q2,2) - pow(quat.q3,2) + pow(quat.q4,2))*RAD_2_DEG;
	}
}


void CalcGyroEuler(float *gyro_data, struct euler_angles* eu_gyro_est)
{
	eu_gyro_est->yaw += gyro_data[2]*SAMPLE_TIME_ICM/1000.0*RAD_2_DEG;
	eu_gyro_est->pitch += gyro_data[1]*SAMPLE_TIME_ICM/1000.0*RAD_2_DEG;
	eu_gyro_est->roll += gyro_data[0]*SAMPLE_TIME_ICM/1000.0*RAD_2_DEG;
}

void CalcAccLinearToEuler(float* accel_data, struct euler_angles* eu_acc_est)
{
	float pitch = atan(accel_data[0]/accel_data[2])* RAD_2_DEG;
	float roll = atan(accel_data[1]/sqrt(pow(accel_data[0],2) + pow(accel_data[2],2))) * RAD_2_DEG;

	eu_acc_est->roll = roll;
	eu_acc_est->pitch = pitch;
}

void CalcGyroQuaternion(float* gyro_data, struct quaternion *q)
{
	struct quaternion q_gyro_rate = {0, gyro_data[0], gyro_data[1], gyro_data[2]};

	q_gyro_rate = q_multiplication(*q, q_gyro_rate);
	q_scalar(&q_gyro_rate,0.5 * SAMPLE_TIME_ICM/1000.0);
	*q = q_add(q_gyro_rate, *q);
	q_normalize(q);
}


void MahonyFilter(float *gyro_data, float* accel_data, struct quaternion *q)
{
	float accelLength;
	float gyro_temp[3];
	float v[3] = {0,0,0};	//corrected frame vector
	float e[3] = {0,0,0};	//error estimate vector
	float i[3] = {0,0,0};	//integral feedback vector

	accelLength = sqrt(accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2]);

	// Normalize accelerometer data
	accel_data[0] /= accelLength;
	accel_data[1] /= accelLength;
	accel_data[2] /= accelLength;

	// Estimated direction of gravity in the body frame
	v[0] = 2.0*(q->q2*q->q4 - q->q1 * q->q3);
	v[1] = 2.0*(q->q1*q->q2 + q->q3 * q->q4);
	v[2] = q->q1*q->q1 - q->q2*q->q2 -q->q3*q->q3 + q->q4 * q->q4;

	e[0] = (accel_data[1] * v[2] - accel_data[2] * v[1]);
	e[1] = (accel_data[2] * v[0] - accel_data[0] * v[2]);
	e[2] = (accel_data[0] * v[1] - accel_data[1] * v[0]);

	// accumulate integral error
	if (k_i > 0.0)
	{
		i[0] += e[0];
		i[1] += e[1];
		i[2] += e[2];
	}

	// Proportionate feedback
	gyro_temp[0] = gyro_data[0] + k_p * e[0] + k_i*i[0];
	gyro_temp[1] = gyro_data[1] + k_p * e[1] + k_i*i[1];
	gyro_temp[2] = gyro_data[2] + k_p * e[2] + k_i*i[2];

	CalcGyroQuaternion(gyro_temp, q);
}

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


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
float i[3] = {0,0,0};
struct matrix RotationMatrix;

/*Madgwick Filter Parameters */
static float Beta = 0.1;

void CalcQuaternionToEuler(struct quaternion quat, struct euler_angles* eu)
{
	eu->roll = 90 - atan2((quat.q1*quat.q2 + quat.q3*quat.q4),0.5 - (quat.q2*quat.q2 + quat.q3*quat.q3))*RAD_2_DEG;
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
	/*
	if (gyro_data[0] < GYRO_THRESHOLD && gyro_data[0] > -GYRO_THRESHOLD)
	{
		gyro_data[0] = 0.0;
	}

	if (gyro_data[1] < GYRO_THRESHOLD && gyro_data[1] > -GYRO_THRESHOLD)
	{
		gyro_data[1] = 0.0;
	}

	if (gyro_data[2] < GYRO_THRESHOLD && gyro_data[2] > -GYRO_THRESHOLD)
	{
		gyro_data[2] = 0.0;
	}
	*/

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

	gyro_data[0] *= DEG_2_RAD;
	gyro_data[1] *= DEG_2_RAD;
	gyro_data[2] *= DEG_2_RAD;

	accelLength = sqrt(accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2]);

	// Normalize accelerometer data
	accel_data[0] /= accelLength;
	accel_data[1] /= accelLength;
	accel_data[2] /= accelLength;

	// Estimated direction of gravity in the body frame
	v[0] = 2.0*(q->q2*q->q4 - q->q1 * q->q3);
	v[1] = 2.0*(q->q1*q->q2 + q->q3 * q->q4);
	v[2] = q->q1*q->q1 - q->q2*q->q2 - q->q3*q->q3 + q->q4 * q->q4;

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

void CalculateRotationMatrix(float* acc_biaz){
	struct matrix Roll;
	struct matrix Pitch;
	struct matrix Yaw;
	struct matrix YawTimesPitch;

	Roll.a11 = 1;
	Roll.a12 = 0;
	Roll.a13 = 0;
	Roll.a21 = 0;
	Roll.a22= cos(acc_biaz[0]);
	Roll.a23 = -sin(acc_biaz[0]);
	Roll.a31 = 0;
	Roll.a32 = sin(acc_biaz[0]);
	Roll.a33 = cos(acc_biaz[0]);

	Pitch.a11 = cos(acc_biaz[1]);
	Pitch.a12 = 0;
	Pitch.a13 = sin(acc_biaz[1]);
	Pitch.a21 = 0;
	Pitch.a22 = 1;
	Pitch.a23 = 0;
	Pitch.a31 = -sin(acc_biaz[1]);
	Pitch.a32 = 0;
	Pitch.a33 = cos(acc_biaz[1]);

	Yaw.a11 = 1;
	Yaw.a12 = 0;
	Yaw.a13 = 0;
	Yaw.a21 = 0;
	Yaw.a22 = 1;
	Yaw.a23 = 0;
	Yaw.a31 = 0;
	Yaw.a32 = 0;
	Yaw.a33 = 1;

	YawTimesPitch.a11 = Yaw.a11*Pitch.a11 + Yaw.a12*Pitch.a21 + Yaw.a13*Pitch.a31;
	YawTimesPitch.a12 = Yaw.a11*Pitch.a12 + Yaw.a12*Pitch.a22 + Yaw.a13*Pitch.a32;
	YawTimesPitch.a13 = Yaw.a11*Pitch.a13 + Yaw.a12*Pitch.a23 + Yaw.a13*Pitch.a33;
	YawTimesPitch.a21 = Yaw.a21*Pitch.a11 + Yaw.a22*Pitch.a21 + Yaw.a23*Pitch.a31;
	YawTimesPitch.a22 = Yaw.a21*Pitch.a12 + Yaw.a22*Pitch.a22 + Yaw.a23*Pitch.a32;
	YawTimesPitch.a23 = Yaw.a21*Pitch.a13 + Yaw.a22*Pitch.a23 + Yaw.a23*Pitch.a33;
	YawTimesPitch.a31 = Yaw.a31*Pitch.a11 + Yaw.a32*Pitch.a21 + Yaw.a33*Pitch.a31;
	YawTimesPitch.a32 = Yaw.a31*Pitch.a12 + Yaw.a32*Pitch.a22 + Yaw.a33*Pitch.a32;
	YawTimesPitch.a33 = Yaw.a31*Pitch.a13 + Yaw.a32*Pitch.a23 + Yaw.a33*Pitch.a33;

	RotationMatrix.a11 = YawTimesPitch.a11*Roll.a11 + YawTimesPitch.a12*Roll.a21 + YawTimesPitch.a13*Roll.a31;
	RotationMatrix.a12 = YawTimesPitch.a11*Roll.a12 + YawTimesPitch.a12*Roll.a22 + YawTimesPitch.a13*Roll.a32;
	RotationMatrix.a13 = YawTimesPitch.a11*Roll.a13 + YawTimesPitch.a12*Roll.a23 + YawTimesPitch.a13*Roll.a33;
	RotationMatrix.a21 = YawTimesPitch.a21*Roll.a11 + YawTimesPitch.a22*Roll.a21 + YawTimesPitch.a23*Roll.a31;
	RotationMatrix.a22 = YawTimesPitch.a21*Roll.a12 + YawTimesPitch.a22*Roll.a22 + YawTimesPitch.a23*Roll.a32;
	RotationMatrix.a23 = YawTimesPitch.a21*Roll.a13 + YawTimesPitch.a22*Roll.a23 + YawTimesPitch.a23*Roll.a33;
	RotationMatrix.a31 = YawTimesPitch.a31*Roll.a11 + YawTimesPitch.a32*Roll.a21 + YawTimesPitch.a33*Roll.a31;
	RotationMatrix.a32 = YawTimesPitch.a31*Roll.a12 + YawTimesPitch.a32*Roll.a22 + YawTimesPitch.a33*Roll.a32;
	RotationMatrix.a33 = YawTimesPitch.a31*Roll.a13 + YawTimesPitch.a32*Roll.a23 + YawTimesPitch.a33*Roll.a33;

}

void CalculateGyroInEarthFrame(float* gyro_data, float* new_data){

	new_data[0] = gyro_data[0]*RotationMatrix.a11 + gyro_data[1]*RotationMatrix.a12 + gyro_data[2]*RotationMatrix.a13;
	new_data[1] = gyro_data[0]*RotationMatrix.a21 + gyro_data[1]*RotationMatrix.a22 + gyro_data[2]*RotationMatrix.a23;
	new_data[2] = gyro_data[0]*RotationMatrix.a31 + gyro_data[1]*RotationMatrix.a32 + gyro_data[2]*RotationMatrix.a33;

	float length = sqrt(pow(new_data[0],2) + pow(new_data[1],2) + pow(new_data[2],2));
	new_data[0] = new_data[0] / length;
	new_data[1] = new_data[1] / length;
	new_data[2] = new_data[2] / length;

}

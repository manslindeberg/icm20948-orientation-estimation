#ifndef SRC_CALC_H_
#define SRC_CALC_H_
#include "math.h"

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

void CalcQuaternionToEuler(struct quaternion quat, struct euler_angles*);
void CalcQuaternionToEuler2(struct quaternion quat, struct euler_angles *eu);
void MahonyFilterXIO(float*, float*, struct quaternion*);
void MadgwickFilterXIO(float *, float*, struct quaternion *);
void MadgwickFilterArduino(float *, float*, struct quaternion *);
void CalcGyroQuaternion(float* gyro_data, struct quaternion*);
void CalcGyroEuler(float *gyro_data, struct euler_angles*);
void CalcAccLinearToEuler(float* accel_data, struct euler_angles*);
void GyroHighPassFilter(float*, float*, float*, float*, float);
void GyroLowPassFilter(float *gyro_data, float* prev_filt, float* filt, float a);
void MahonyFilter(float*, float*, struct quaternion *);

struct quaternion q_multiplication(struct quaternion quad_left, struct quaternion quad_right);
struct quaternion q_add(struct quaternion quad_left, struct quaternion quad_right);
struct quaternion q_subtract(struct quaternion quad_left, struct quaternion quad_right);
struct quaternion q_inverse(struct quaternion quad);

void q_normalize(struct quaternion * quad);
void q_scalar(struct quaternion *quad, float s);

#endif /* SRC_CALC_H_ */

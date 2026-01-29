#ifndef GEN_DATA_H
#define GEN_DATA_H

#include "../ahrs.h"

struct imu_data {
	vector_struct gyro = {0, 0, 0};
	vector_struct accel = {0, 0, 0};
	vector_struct mag = {0, 0, 0};
};

struct sim_settings {
	double dt = 0.0015;

	double gyro_drift = 0;

	double accel_noise = 0.1;
};

// Simulates rotation to generate fake IMU data.
imu_data gen_data();

#endif

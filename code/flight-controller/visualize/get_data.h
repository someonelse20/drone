#ifndef GET_DATA_H
#define GET_DATA_H

extern "C" {

struct vector {
	float x, y, z;
};

// Initialize LSM9DS1.
void imu_init(int accel_scale = 8, int gyro_scale = 500, int mag_scale = 0); // Set default settings for LSM9DS1. mag_scale is not currently used.

// Reads IMU and output imu_data struct.
vector read_gyro();

vector read_accel();

vector read_mag();

}

#endif

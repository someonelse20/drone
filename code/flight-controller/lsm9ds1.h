#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <string>
#include "ahrs.h"

struct imu_data {
	vector_struct gyro = {0, 0, 0};
	vector_struct accel = {0, 0, 0};
	vector_struct mag = {0, 0, 0};
};

struct imu_settings {
	int gyro_scale = 250; // dps
	int accel_scale = 2; // g
	int mag_scale = 4; // gauss

	int data_rate = 6; // 0 - 6
	int mag_data_rate = 7; // 0 - 7

	int mag_mode = 3; // 0 = low power, 1 = med performance, 2 = high performance, 3 = ultra performance
	bool mag_temp_comp = 0; // Magnetometer temperature compensation enable.
};

class lsm9ds1 {
private:
	int handle;
	int m_handle;

	int ADDR;
	int MAG_ADDR;
	int BUS;

	imu_data data;

	int gyro_scale;
	int accel_scale;
	int mag_scale;

	int data_rate;
	int mag_data_rate;

	// Arrays to make settings easier.
	std::string gyro_scales[3] = {"00", "01", "11"};
	std::string accel_scales[4] = {"00", "10", "11", "01"};
	std::string mag_scales[4] = {"00", "01", "10", "11"};

	std::string data_rates[7] = {"000", "001", "010", "011", "100", "101", "110"};
	std::string mag_data_rates[8] = {"000", "001", "010", "011", "100", "101", "110", "111"};
	std::string mag_modes[4] = {"00", "01", "10", "11"};

	// Arrays to store sensor sensitivities to convert raw values.
	double gyro_sensitivities[3] = {8.75, 17.50, 70}; 
	double accel_sensitivities[4] = {0.061, 0.122, 0.244, 0.732};
	double mag_sensitivities[4] = {0.14, 0.29, 0.43, 0.58};

public:
	imu_settings settings;

	lsm9ds1(int addr, int mag_addr, int bus = 1);

	void init();

	imu_data read();

	// void test_imu();
};

#endif

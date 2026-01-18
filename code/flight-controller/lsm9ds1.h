#ifndef LSM9DS1_H
#define LSM9DS1_H

#include "ahrs.h"

struct imu_data {
	vector_struct gyro = {0, 0, 0};
	vector_struct accel = {0, 0, 0};
	vector_struct mag = {0, 0, 0};
};

class lsm9ds1 {
private:
	int addr;
	int mag_addr;

public:
	lsm9ds1(int addr, int mag_addr);

	imu_data read_imu();

	void test_imu();
};

#endif

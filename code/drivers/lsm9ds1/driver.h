#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"

struct data {
	float gyro[3], accel[3], mag[3];
};

int x = 0, y = 1, z = 3;

LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

void init() {
    imu.begin();
    if (!imu.begin()) {
        fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        exit(EXIT_FAILURE);
    }
    imu.calibrate();
}

data get_data() {
	data return_data;
	while (!imu.gyroAvailable()) 
		;
    imu.readGyro();
    while(!imu.accelAvailable()) 
		;
    imu.readAccel();
    while(!imu.magAvailable()) 
		;
    imu.readMag();

	return_data.gyro[x] = imu.calcGyro(imu.gx);
	return_data.gyro[y] = imu.calcGyro(imu.gy);
	return_data.gyro[z] = imu.calcGyro(imu.gz);
	
	return_data.accel[x] = imu.calcGyro(imu.ax);
	return_data.accel[y] = imu.calcGyro(imu.ay);
	return_data.accel[z] = imu.calcGyro(imu.az);

	return_data.mag[x] = imu.calcGyro(imu.mx);
	return_data.mag[y] = imu.calcGyro(imu.my);
	return_data.mag[z] = imu.calcGyro(imu.mz);

	return return_data;
}


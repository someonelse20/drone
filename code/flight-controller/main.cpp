#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include "ahrs.h"
#include <LSM9DS1_Types.h>
#include "LSM9DS1.h"

using namespace std;

struct imu_data {
	vector<double> gyro = {0, 0, 0};
	vector<double> accel = {0, 0, 0};
	vector<double> mag = {0, 0, 0};
};

LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

double dt = 10; // Deltatime in miliseconds.

// Initialize LSM9DS1.
void imu_init(int accel_scale = 4, int gyro_scale = 500, int mag_scale = 0) { // Set default settings for LSM9DS1. mag_scale is not currently used.
	// Set settings
	imu.settings.gyro.scale = gyro_scale;
	imu.settings.accel.scale = accel_scale;
	//imu.settings.mag.scale = mag_scale;

	imu.begin();
	if (!imu.begin()) {
		cout << "Failed to communitate to LSM9DS1." << endl;
		exit(EXIT_FAILURE);
	}
	// Possibly remove later.
	imu.calibrate();
};

// Read data from LSM9DS1 and output the gyro, accel, and mag data in a struct.
imu_data read_imu() {
	imu_data data;

	while (!imu.gyroAvailable()) ;
	imu.readGyro();

	while (!imu.accelAvailable()) ;
	imu.readAccel();

	while (!imu.magAvailable()) ;
	imu.readMag();

	data.gyro[0] = imu.calcGyro(imu.gx);
	data.gyro[1] = imu.calcGyro(imu.gy);
	data.gyro[2] = imu.calcGyro(imu.gz);

	data.accel[0] = imu.calcAccel(imu.gx);
	data.accel[1] = imu.calcAccel(imu.gy);
	data.accel[2] = imu.calcAccel(imu.gz);

	data.mag[0] = imu.calcMag(imu.gx);
	data.mag[1] = imu.calcMag(imu.gy);
	data.mag[2] = imu.calcMag(imu.gz);

	return data;
}

// Prints LSM9DS1 readings to stout.
void test_imu(bool loop=false) { // If loop = true then this function will loop while true. loop = false to disable (default).
	imu_init();

	imu_data data;

	if (loop) {
		while (1) {
			data = read_imu();

			cout << "Gyro: " << to_string(data.gyro[0]) + "," + to_string(data.gyro[1]) + "," + to_string(data.gyro[2]) << ", Accel: " << to_string(data.accel[0]) + "," + to_string(data.accel[1]) + "," + to_string(2) << ", Mag: " << to_string(data.mag[0]) + "," + to_string(data.mag[1]) + "," + to_string(data.mag[2]) << endl;
		
			usleep(100000);
		}
	} else {
		data = read_imu();

		cout << "Gyro: " << to_string(data.gyro[0]) + "," + to_string(data.gyro[1]) + "," + to_string(data.gyro[2]) << ", Accel: " << to_string(data.accel[0]) + "," + to_string(data.accel[1]) + "," + to_string(2) << ", Mag: " << to_string(data.mag[0]) + "," + to_string(data.mag[1]) + "," + to_string(data.mag[2]) << endl;
		
		usleep(100000);
	}
}

// Updates orientation using ahrs algoritm. See ahrs.h and ahrs.cpp.
void update_orientation(ahrs ahrs_alg, imu_data data) {
	ahrs_alg.update(data.gyro, data.accel, data.mag, dt);

	return;
}

int main() {
	imu_init();

	// Set ahrs settings.
	ahrs ahrs_alg;

	while (true) {
		update_orientation(ahrs_alg, read_imu());
	}
}


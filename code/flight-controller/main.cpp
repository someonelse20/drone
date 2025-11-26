#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>
#include <chrono>
#include "ahrs.h"
#include <LSM9DS1_Types.h>
#include "LSM9DS1.h"

using namespace std;

struct imu_data {
	vector_struct gyro = {0, 0, 0};
	vector_struct accel = {0, 0, 0};
	vector_struct mag = {0, 0, 0};
};

LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

double dt = 0.027; // Deltatime in miliseconds.

// Gets time in miliseconds since epoch.
double get_timestamp() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// Initialize LSM9DS1.
void imu_init(int accel_scale = 8, int gyro_scale = 500, int mag_scale = 0) { // Set default settings for LSM9DS1. mag_scale is not currently used.
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
	// imu.calibrate();
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

	data.gyro.x = imu.calcGyro(imu.gx);
	data.gyro.y = imu.calcGyro(imu.gy);
	data.gyro.z = imu.calcGyro(imu.gz);

	data.accel.x = imu.calcAccel(imu.ax);
	data.accel.y = imu.calcAccel(imu.ay);
	data.accel.z = imu.calcAccel(imu.az);

	data.mag.x = imu.calcMag(imu.mx);
	data.mag.y = imu.calcMag(imu.my);
	data.mag.z = imu.calcMag(imu.mz);

	return data;
}

// Prints LSM9DS1 readings to stout.
void test_imu(bool loop=false) { // If loop = true then this function will loop while true. loop = false to disable (default).
	imu_init();

	imu_data data;

	if (loop) {
		while (1) {
			data = read_imu();

			cout << "Gyro: " << to_string(data.gyro.x) + "," + to_string(data.gyro.y) + "," + to_string(data.gyro.z) << ", Accel: " << to_string(data.accel.x) + "," + to_string(data.accel.y) + "," + to_string(data.accel.z) << ", Mag: " << to_string(data.mag.x) + "," + to_string(data.mag.y) + "," + to_string(data.mag.z) << endl;
		
			usleep(100000);
		}
	} else {
		data = read_imu();

			cout << "Gyro: " << to_string(data.gyro.x) + "," + to_string(data.gyro.y) + "," + to_string(data.gyro.z) << ", Accel: " << to_string(data.accel.x) + "," + to_string(data.accel.y) + "," + to_string(data.accel.z) << ", Mag: " << to_string(data.mag.x) + "," + to_string(data.mag.y) + "," + to_string(data.mag.z) << endl;

		usleep(100000);
	}
}

// Updates orientation using ahrs algoritm. See ahrs.h and ahrs.cpp.
output_struct update_orientation(ahrs ahrs_alg, imu_data data) {
	return ahrs_alg.update(data.gyro, data.accel, data.mag, dt);
}

int main() {
	imu_init();

	// Set ahrs settings.
	ahrs ahrs_alg;

	ahrs_alg.settings.gyro_calibrate.bias = {0.75393149, 1.16563878, -8.15698371};
	ahrs_alg.settings.gyro_calibrate.sensitivity = {0.82820539, 0.86249338, 0.85730546};
	ahrs_alg.settings.gyro_calibrate.rotation_matrix = {{347, 8, 13},
                                                        {4, 355, 1},
                                                        {0, 10, 347}};

	ahrs_alg.settings.gain_normal = 0.0000001;

	/*
	quaternion_struct product = ahrs_alg.quaternion_product(quaternion_struct {3, 2, 4, 1}, quaternion_struct {1, 3, 5, 2});
	cout << product.w << "," << product.x << "," << product.y << "," << product.z << endl;
	*/

	// test_imu(true);

	while (true) {
		double timestamp = get_timestamp();

		imu_data imu_readings = read_imu();

		output_struct orientation = ahrs_alg.update(imu_readings.gyro, imu_readings.accel, imu_readings.mag, dt);

		cout << to_string(orientation.orientation.euler.x) + ", " + to_string(orientation.orientation.euler.y) + ", " + to_string(orientation.orientation.euler.z) << endl;
		// cout << orientation.orientation.quaterion.x << "," << orientation.orientation.quaterion.y << "," << orientation.orientation.quaterion.z << "," << orientation.orientation.quaterion.w << endl;
		// cout << get_timestamp() - timestamp << endl;
	}
}


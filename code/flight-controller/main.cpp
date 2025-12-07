#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <cmath>
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

double dt = 0.013; // Deltatime in seconds.

// Gets time in miliseconds since epoch.
double get_timestamp() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// Initialize LSM9DS1.
void imu_init(int accel_scale = 2, int gyro_scale = 245, int mag_scale = 4) { // Set default settings for LSM9DS1.
	// Set settings
	imu.settings.gyro.scale = gyro_scale;
	imu.settings.accel.scale = accel_scale;
	imu.settings.mag.scale = mag_scale;

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

vector_struct gyro_calibrate() {
	return read_imu().gyro;
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

int main() {
	imu_init();

	// Set ahrs settings.
	ahrs ahrs_alg;

	ahrs_alg.settings.gyro_calibrate.bias = ahrs_alg.gyro_bias_calibration(1, &gyro_calibrate); //{2.93716433, 0.08483891, 0.71578982};
	ahrs_alg.settings.gyro_calibrate.sensitivity = {0.8409687, 0.87876116, 0.87530723};
	// ahrs_alg.settings.gyro_calibrate.rotation_matrix = {{1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}};
	
	ahrs_alg.settings.accel_calibrate.bias = {-0.04, 0, 0};
	// ahrs_alg.settings.accel_calibrate.bias = {-0.02425592, 0.00482671, 0.00482671};
	// ahrs_alg.settings.accel_calibrate.sensitivity = {1.00154462, 0.99859621, 0.99785103};
	// ahrs_alg.settings.accel_calibrate.rotation_matrix = {{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}};

	ahrs_alg.settings.mag_calibrate.hard_iorn = {-0.03810718,  0.18329252, -0.13790179};
	ahrs_alg.settings.mag_calibrate.soft_iorn = {{1.04397915e+00, 6.24500451e-17, 2.77555756e-17},
                                                 {6.24500451e-17, 1.04397915e+00, 9.43689571e-16},
                                                 {0.00000000e+00, 9.43689571e-16, 1.04397915e+00}};
	// ahrs_alg.settings.mag_calibrate.rotation_matrix = {{0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}};

	ahrs_alg.settings.accel_rejection_t = 0.5;

	ahrs_alg.settings.gain_normal = 5.0;
	ahrs_alg.settings.gain_init = 15.0;

	ahrs_alg.settings.declination = 133.3;
	ahrs_alg.settings.add_declination = false;

	// test_imu(true);

	while (true) {
		// double timestamp = ahrs_alg.get_timestamp();

		imu_data imu_readings = read_imu();

		// cout << imu_readings.accel.x << "," << imu_readings.accel.y << "," << imu_readings.accel.z << endl;

		// cout << imu_readings.accel.z << endl;;

		// output_struct orientation = ahrs_alg.update(imu_readings.gyro, imu_readings.accel, {0, 0, 0}, dt);
		output_struct orientation = ahrs_alg.update(imu_readings.gyro, imu_readings.accel, imu_readings.mag, dt);

		// cout << orientation.orientation.euler.z << endl;
		// cout << to_string(orientation.orientation.euler.x) + ", " + to_string(orientation.orientation.euler.y) + ", " + to_string(orientation.orientation.euler.z) << endl;
		// cout << orientation.orientation_earth_frame.euler.x << ", " << orientation.orientation_earth_frame.euler.y << ", " << orientation.orientation_earth_frame.euler.z << endl;
		// cout << orientation.orientation.quaterion.x << "," << orientation.orientation.quaterion.y << "," << orientation.orientation.quaterion.z << "," << orientation.orientation.quaterion.w << endl;
		// cout << orientation.acceleration.global.x << "," << orientation.acceleration.global.y << "," << orientation.acceleration.global.z << endl;
		// cout << orientation.acceleration.zero.x << ", " << orientation.acceleration.zero.y << ", " << orientation.acceleration.zero.z << endl;

		double heading = orientation.orientation.euler.z;
		if (heading < 0) heading += 360;
		heading += 15;
		cout << heading << endl;

		// cout << ahrs_alg.get_timestamp() - timestamp << endl;
	}
}


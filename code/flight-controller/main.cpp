#include <cstdlib>
#include <iostream>
#include <pigpio.h>
#include <string>
#include <unistd.h>
#include <chrono>
#include <cmath>
/*
#include <LSM9DS1_Types.h>
#include "LSM9DS1.h"
*/
#include "lsm9ds1.h"
#include "motor.h"
#include "ahrs.h"
#include "pid.h"

// TODO: Add threads that join at the end of each loop.
// For example:
// read_imu_thread.join()
// ahrs_thread.join()
// pid_thread.join()
// set_motor_thread.join()

// TODO: Add json config file.

using namespace std;

/*
struct imu_data {
	vector_struct gyro = {0, 0, 0};
	vector_struct accel = {0, 0, 0};
	vector_struct mag = {0, 0, 0};
};

LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);
*/
lsm9ds1 imu(0x6b, 0x1e, 1);

double dt = 0.002; // Deltatime in seconds.

double throttle = 30; 
double set_x = 0, set_y = 0, set_z = 0; // The angles the flight controler will try to stay at.

// Gets time in miliseconds since epoch.
double get_timestamp() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// Clamp a value between two values.
double clamp(double value, double min, double max) {
	if (value < min) {
		return min;
	}
	else if (value > max) {
		return max;
	}
	else {
		return value;
	}
}

/*
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

	cout << imu.ax << endl;

	data.mag.x = imu.calcMag(imu.mx);
	data.mag.y = imu.calcMag(imu.my);
	data.mag.z = imu.calcMag(imu.mz);

	return data;
}
*/

void imu_init() {


}

vector_struct gyro_calibrate() {
	return imu.read().gyro;
}

// Prints LSM9DS1 readings to stout.
void test_imu(bool loop=false) { // If loop = true then this function will loop while true. loop = false to disable (default).
	imu_init();

	imu_data data;

	if (loop) {
		while (1) {
			data = imu.read();

			cout << "Gyro: " << to_string(data.gyro.x) + "," + to_string(data.gyro.y) + "," + to_string(data.gyro.z) << ", Accel: " << to_string(data.accel.x) + "," + to_string(data.accel.y) + "," + to_string(data.accel.z) << ", Mag: " << to_string(data.mag.x) + "," + to_string(data.mag.y) + "," + to_string(data.mag.z) << endl;
		
			usleep(100000);
		}
	} else {
		data = imu.read();

			cout << "Gyro: " << to_string(data.gyro.x) + "," + to_string(data.gyro.y) + "," + to_string(data.gyro.z) << ", Accel: " << to_string(data.accel.x) + "," + to_string(data.accel.y) + "," + to_string(data.accel.z) << ", Mag: " << to_string(data.mag.x) + "," + to_string(data.mag.y) + "," + to_string(data.mag.z) << endl;

		usleep(100000);
	}
}

int main() {
	// Initialize GPIO.
	if (gpioInitialise() < 0)
		exit(-1);

	// Set IMU Settings
	imu_settings imu_settings;

	imu_settings.gyro_scale = 500;
	imu_settings.accel_scale = 4;

	// imu_settings.data_rate = 476;
	imu_settings.data_rate = 952;

	imu.init();

	// Construct motors.
	motor front_left_motor(12, 1300, 1520);
	motor front_right_motor(13, 1145, 1510);
	motor back_left_motor(20, 1145, 1515);
	motor back_right_motor(21, 1290, 1530);

	front_left_motor.stop();
	front_right_motor.stop();
	back_left_motor.stop();
	back_right_motor.stop();

	// Construct PID.
	pid_controller pid_x(dt, 1, 0, 0);
	pid_controller pid_y(dt, 1, 0, 0);
	pid_controller pid_z(dt, 0.1, 0, 0);

	// Set ahrs settings.
	ahrs ahrs_alg;

	ahrs_alg.settings.gyro_calibrate.bias = ahrs_alg.gyro_bias_calibration(1, &gyro_calibrate); //{2.93716433, 0.08483891, 0.71578982};
	ahrs_alg.settings.gyro_calibrate.sensitivity = {0.8409687, 0.87876116, 0.87530723};

	ahrs_alg.settings.accel_calibrate.bias = {-0.04030407, 0.01214501, -0.02037599};
	ahrs_alg.settings.accel_calibrate.sensitivity = {1.0042335, 0.99896223, 0.99228542};

	ahrs_alg.settings.mag_calibrate.soft_iorn = {{ 0.90782961, -0.05349811,  0.0207239 },
                                                 {-0.11265483, -0.91579853,  0.02113201},
                                                 { 0.06981889, -0.12123726, -0.91697565}};
	ahrs_alg.settings.mag_calibrate.hard_iorn = {-0.00618111, -0.12120257, -0.15837325};
	ahrs_alg.settings.mag_calibrate.rotation_matrix = {{ 0,  1,  0},
                                                       {-1,  0,  0},
                                                       { 0,  0,  1}};

	ahrs_alg.settings.accel_rejection_t = 0.5;

	ahrs_alg.settings.gain_normal = 10.0;
	ahrs_alg.settings.gain_init = 20.0;

	ahrs_alg.settings.declination = 133.3;
	ahrs_alg.settings.add_declination = false;

	// Wait until ESCs are initialized.
	// sleep(5);

	front_left_motor.set_speed(1);
	front_right_motor.set_speed(1);
	back_left_motor.set_speed(1);
	back_right_motor.set_speed(1);

	while (true) {
		double timestamp = ahrs_alg.get_timestamp(1);

		// Read from IMU.
		imu_data imu_readings = imu.read();

		// cout << imu_readings.accel.x << ", " << imu_readings.accel.y << ", " << imu_readings.accel.z << endl; 

		// cout << (ahrs_alg.get_timestamp() - timestamp) * 1000 << endl;

		// Get orientation.
		output_struct ahrs_output = ahrs_alg.update(imu_readings.gyro, imu_readings.accel, imu_readings.mag, dt);
		// cout << ahrs_output.orientation.euler.x << ", " << ahrs_output.orientation.euler.y << ", " << ahrs_output.orientation.euler.z << endl;

		// Update PID controlers (curently set to maintain orientation).
		double pid_x_output = pid_x.loop(ahrs_output.orientation.euler.x, set_x);
		double pid_y_output = pid_y.loop(ahrs_output.orientation.euler.y, set_y);
		double pid_z_output = pid_z.loop(ahrs_output.orientation.euler.z, set_z);

		// cout << pid_x_output << ", " << pid_y_output << ", " << pid_z_output << endl;

		// Set motor speeds.
		// /*
		front_left_motor.set_speed(clamp(throttle + pid_x_output + pid_y_output - pid_z_output, 5, 100));
		front_right_motor.set_speed(clamp(throttle - pid_x_output + pid_y_output + pid_z_output, 5, 100));
		back_left_motor.set_speed(clamp(throttle + pid_x_output - pid_y_output + pid_z_output, 5, 100));
		back_right_motor.set_speed(clamp(throttle - pid_x_output - pid_y_output - pid_z_output, 5, 100));
		// */


		// cout << clamp(throttle + pid_x_output - pid_y_output + pid_z_output, 0, 100) << endl;

		// Make dt constant.
		double time_elapsed = ahrs_alg.get_timestamp(1) - timestamp;

		// cout << dt * 1000000 - time_elapsed << endl;

		usleep(clamp(dt - time_elapsed, 0, dt * 1000000));
	}
}


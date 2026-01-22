#include <nlohmann/json.hpp>
#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <fstream>
#include <cstdlib>
#include <string>
#include <chrono>
#include <cmath>

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

using json = nlohmann::json;
using namespace std;

double dt = 0.0015; // Deltatime in seconds.

double throttle = 30; 
double set_x = 0, set_y = 0, set_z = 0; // The angles the flight controler will try to stay at.

lsm9ds1 imu(0x6b, 0x1e, 1);

// Construct motors.
motor front_left_motor(12, 1300, 1520);
motor front_right_motor(13, 1145, 1510);
motor back_left_motor(20, 1145, 1515);
motor back_right_motor(21, 1290, 1530);

// Construct PID.
pid_controller pid_x(dt, 1, 0, 0);
pid_controller pid_y(dt, 1, 0, 0);
pid_controller pid_z(dt, 0.1, 0, 0);

// Construct ahrs.
ahrs ahrs_alg;

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

// Converts json list to vector_struct.
vector_struct json_to_vector(json value) {
	return {value[0], value[1], value[2]};
}

// Converts json list to matrix_struct.
matrix_struct json_to_matrix(json value) {
	return {{value[0][0], value[0][1], value[0][2]},
            {value[1][0], value[1][1], value[1][2]},
            {value[2][0], value[2][1], value[2][2]}};
}

vector_struct gyro_calibrate() {
	return imu.read().gyro;
}

// Prints LSM9DS1 readings to stout.
void test_imu(bool loop=false) { // If loop = true then this function will loop while true. loop = false to disable (default).
	imu.init();

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

void set_settings() {
	fstream file;

	file.open("../config.json");

	json config = json::parse(file);

	// AHRS settings.
	ahrs_alg.settings.gain_normal = config["ahrs"]["gain_normal"];
	ahrs_alg.settings.gain_init = config["ahrs"]["gain_init"];
	ahrs_alg.settings.init_time = config["ahrs"]["init_time"];

	ahrs_alg.settings.min_mag_distortion = config["ahrs"]["min_mag_distortion"];
	ahrs_alg.settings.max_mag_distoriton = config["ahrs"]["max_mag_distoriton"];
	ahrs_alg.settings.declination = config["ahrs"]["declination"];
	ahrs_alg.settings.add_declination = config["ahrs"]["add_declination"];

	ahrs_alg.settings.accel_rejection = config["ahrs"]["accel_rejection"];
	ahrs_alg.settings.accel_rejection_t = config["ahrs"]["accel_rejection_t"];

	ahrs_alg.settings.gyro_calibrate.sensitivity = json_to_vector(config["ahrs"]["gyro_calibrate"]["sensitivity"]);

	ahrs_alg.settings.accel_calibrate.bias = json_to_vector(config["ahrs"]["accel_calibrate"]["bias"]);
	ahrs_alg.settings.accel_calibrate.sensitivity = json_to_vector(config["ahrs"]["accel_calibrate"]["sensitivity"]);

	ahrs_alg.settings.mag_calibrate.hard_iorn = json_to_vector(config["ahrs"]["mag_calibrate"]["hard_iorn"]);
	ahrs_alg.settings.mag_calibrate.soft_iorn = json_to_matrix(config["ahrs"]["mag_calibrate"]["soft_iorn"]);
	ahrs_alg.settings.mag_calibrate.rotation_matrix = json_to_matrix(config["ahrs"]["mag_calibrate"]["rotation_matrix"]);

	// IMU settings.
	imu.settings.gyro_scale = config["imu"]["gyro_scale"];
	imu.settings.accel_scale = config["imu"]["accel_scale"];
	imu.settings.mag_scale = config["imu"]["mag_scale"];

	imu.settings.data_rate = config["imu"]["data_rate"];
	imu.settings.mag_data_rate = config["imu"]["mag_data_rate"];

	imu.settings.mag_mode = config["imu"]["mag_mode"];
	imu.settings.mag_temp_comp = config["imu"]["mag_temp_comp"];

	// Motor settings.
	front_left_motor.GPIO = config["motors"]["front_left"]["pin"];
	front_left_motor.min_speed = config["motors"]["front_left"]["min"];
	front_left_motor.max_speed = config["motors"]["front_left"]["max"];

	front_right_motor.GPIO = config["motors"]["front_right"]["pin"];
	front_right_motor.min_speed = config["motors"]["front_right"]["min"];
	front_right_motor.max_speed = config["motors"]["front_right"]["max"];

	back_left_motor.GPIO = config["motors"]["back_left"]["pin"];
	back_left_motor.min_speed = config["motors"]["back_left"]["min"];
	back_left_motor.max_speed = config["motors"]["back_left"]["max"];

	back_right_motor.GPIO = config["motors"]["back_right"]["pin"];
	back_right_motor.min_speed = config["motors"]["back_right"]["min"];
	back_right_motor.max_speed = config["motors"]["back_right"]["max"];

	// PID settings.
	pid_x.kP = config["pid"]["x"]["kP"];
	pid_x.kI = config["pid"]["x"]["kI"];
	pid_x.kD = config["pid"]["x"]["kD"];
	pid_x.gain = config["pid"]["x"]["gain"];

	pid_y.kP = config["pid"]["y"]["kP"];
	pid_y.kI = config["pid"]["y"]["kI"];
	pid_y.kD = config["pid"]["y"]["kD"];
	pid_y.gain = config["pid"]["y"]["gain"];

	pid_z.kP = config["pid"]["z"]["kP"];
	pid_z.kI = config["pid"]["z"]["kI"];
	pid_z.kD = config["pid"]["z"]["kD"];
	pid_z.gain = config["pid"]["z"]["gain"];
}

int main() {
	// Initialize GPIO.
	if (gpioInitialise() < 0)
		exit(-1);

	front_left_motor.stop();
	front_right_motor.stop();
	back_left_motor.stop();
	back_right_motor.stop();

	set_settings();

	/*
	// Set IMU Settings
	imu_settings imu_settings;

	imu_settings.gyro_scale = 500;
	imu_settings.accel_scale = 4;

	// imu_settings.data_rate = 6;

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
	*/

	imu.init();

	// Wait until ESCs are initialized.
	// sleep(5);
	sleep(1);

	// Calibrate gyro bias.
	ahrs_alg.settings.gyro_calibrate.bias = ahrs_alg.gyro_bias_calibration(1, &gyro_calibrate);

	front_left_motor.set_speed(1);
	front_right_motor.set_speed(1);
	back_left_motor.set_speed(1);
	back_right_motor.set_speed(1);

	while (true) {
		double timestamp = ahrs_alg.get_timestamp(1);

		// Read from IMU.
		imu_data imu_readings = imu.read();

		// cout << imu_readings.accel.x << ", " << imu_readings.accel.y << ", " << imu_readings.accel.z << endl; 

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

		// cout << time_elapsed << endl;
		// cout << dt * 1000000 - time_elapsed << endl;

		usleep(clamp(dt - time_elapsed, 0, dt * 1000000));
	}
}


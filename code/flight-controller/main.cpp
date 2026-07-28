#include <cstdio>
#include <nlohmann/json.hpp>
#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <fstream>
#include <cstdlib>
#include <string>
#include <cmath>

#include "lsm9ds1.h"
#include "motor.h"
#include "ahrs/ahrs.h"
#include "pid.h"

// TODO: Add threads that join at the end of each loop.
// For example:
// read_imu_thread.join()
// ahrs_thread.join()
// pid_thread.join()
// set_motor_thread.join()

using json = nlohmann::json;
using namespace std;

double dt = 0.0015; // Deltatime in seconds.

double throttle = 10; 
double set_x = 0, set_y = 0, set_z = 0; // The angles the flight controler will try to stay at.

int flight_id;

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
void test_imu(bool loop=true) { // If loop = true then this function will loop while true. loop = false to disable (default).
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

	file.close();

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

	// Set flight id.
	flight_id = config["flight_id"];
	flight_id++;
	config["flight_id"] = flight_id;

	// Write new flight id.
	// /*
	ofstream write_file;
	write_file.open("../config.json", ofstream::out | ofstream::trunc);

	write_file << config;

	write_file.close();
	// */
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

	imu.init();

	// Wait until ESCs are initialized.
	sleep(5);
	// usleep(1000);
	
	// Calibrate gyro bias.
	ahrs_alg.settings.gyro_calibrate.bias = ahrs_alg.gyro_bias_calibration(1, &gyro_calibrate);

	// /*
	front_left_motor.set_speed(1);
	front_right_motor.set_speed(1);
	back_left_motor.set_speed(1);
	back_right_motor.set_speed(1);
	// */

	// Create log file.
	// ofstream log("../flight_logs/foobar.csv");
	ofstream log("../flight_logs/bench_tests/flight-log_" + to_string(flight_id) + ".csv");
	log << "pid_x,pid_y,pid_z";
	log << ",front_left_speed,front_right_speed,back_left_speed,back_right_speed";
	log << ",orientation_x,orientation_y,orientation_z";
	log << ",acceleration_x,acceleration_y,acceleration_z";
	log << ",accel_rejected,mag_rejected,ahrs_initialized";
	log << ",gyro_x,gyro_y,gyro_z";
	log << ",accel_x,accel_y,accel_z";
	log << ",mag_x,mag_y,mag_z" << endl;

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
		// double pid_y_output = pid_y.loop(ahrs_output.orientation.euler.y, set_y);
		// double pid_z_output = pid_z.loop(ahrs_output.orientation.euler.z, set_z);
		double pid_y_output = 0, pid_z_output = 0;
		// double pid_z_output = 0;

		// cout << pid_x_output << ", " << pid_y_output << ", " << pid_z_output << endl;

		// cout << ahrs_output.initialized << ",";
		// cout << pid_x_output << ",";
		cout << ahrs_output.orientation.euler.x << ",";

		// Set motor speeds.
		/*
		front_left_motor.set_speed(clamp(throttle + pid_x_output + pid_y_output - pid_z_output, 5, 100));
		front_right_motor.set_speed(clamp(throttle - pid_x_output + pid_y_output + pid_z_output, 5, 100));
		back_left_motor.set_speed(clamp(throttle + pid_x_output - pid_y_output + pid_z_output, 5, 100));
		back_right_motor.set_speed(clamp(throttle - pid_x_output - pid_y_output - pid_z_output, 5, 100));
		*/
		// /*
		if (ahrs_output.initialized) {
			front_left_motor.set_speed(clamp(throttle - pid_x_output + pid_y_output - pid_z_output, 1, 100));
			front_right_motor.set_speed(clamp(throttle - pid_x_output - pid_y_output + pid_z_output, 1, 100));
			back_left_motor.set_speed(clamp(throttle + pid_x_output + pid_y_output + pid_z_output, 1, 100));
			back_right_motor.set_speed(clamp(throttle + pid_x_output - pid_y_output - pid_z_output, 1, 100));
		}
		// */

		cout << clamp(throttle - pid_x_output + pid_y_output - pid_z_output, 1, 100);
		cout << ",";
		cout << clamp(throttle - pid_x_output - pid_y_output + pid_z_output, 1, 100);
		cout << ",";
		cout << clamp(throttle + pid_x_output + pid_y_output + pid_z_output, 1, 100);
		cout << ",";
		cout << clamp(throttle + pid_x_output - pid_y_output - pid_z_output, 1, 100) << endl;

		// Write to log file.
		log << pid_x_output << "," << pid_y_output << "," << pid_z_output;
		log << "," << clamp(throttle - pid_x_output + pid_y_output - pid_z_output, 1, 100);
		log << "," << clamp(throttle - pid_x_output - pid_y_output + pid_z_output, 1, 100);
		log << "," << clamp(throttle + pid_x_output + pid_y_output + pid_z_output, 1, 100);
		log << "," << clamp(throttle + pid_x_output - pid_y_output - pid_z_output, 1, 100);
		log << "," << ahrs_output.orientation.euler.x << "," << ahrs_output.orientation.euler.y << "," << ahrs_output.orientation.euler.z;
		log << "," << ahrs_output.acceleration.zero.x << "," << ahrs_output.acceleration.zero.y << "," << ahrs_output.acceleration.zero.z;
		log << "," << ahrs_output.accel_rejected << "," << ahrs_output.mag_rejected << "," << ahrs_output.initialized;
		log << "," << imu_readings.gyro.x << "," << imu_readings.gyro.y << "," << imu_readings.gyro.z; 
		log << "," << imu_readings.accel.x << "," << imu_readings.accel.y << "," << imu_readings.accel.z; 
		log << "," << imu_readings.mag.x << "," << imu_readings.mag.y << "," << imu_readings.mag.z << endl; 

		// log.close();

		// Make dt constant.
		double time_elapsed = ahrs_alg.get_timestamp(1) - timestamp;

		// cout << time_elapsed << ",";
		// cout << dt * 1000000 - time_elapsed << endl;

		usleep(clamp(dt * 1000000 - time_elapsed, 0, dt * 1000000));
		// cout << clamp(dt * 1000000 - time_elapsed, 0, dt * 1000000) << ",";
		// cout << (ahrs_alg.get_timestamp(1) - timestamp) << ",";
	}
}


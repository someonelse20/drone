#include <cstdlib>
#include <iostream>
#include <unistd.h>

#include "../ahrs/ahrs.h"
#include "ahrs_test.h"

using namespace std;

ahrs alg;

int main() {
	set_settings();

	sim_settings sets;

	sets.runtime = 1;
}

void linear_sim(ahrs &alg, sim_settings settings) {
	double timestamp = alg.get_timestamp(1);

	cout << "starting!" << endl;
	while (alg.get_timestamp(1) < timestamp + (settings.runtime * 1000000)) {
		double dt_timestamp = alg.get_timestamp(1);

		imu_data data = {0, 0, 0};

		output_struct output = alg.update(data.gyro, data.accel, data.mag, dt);

		// Make dt constant.
		double time_elapsed = alg.get_timestamp(1) - dt_timestamp;

		usleep(clamp(dt * 1000000 - time_elapsed, 0, dt * 1000000));
	}
}

double rnd() {
	return rand() % 1000000 / 1000000.0;
}

quaternion_struct rnd_quaternion(bool norm) {
	quaternion_struct return_value;

	return_value.w = rnd();
	return_value.x = rnd();
	return_value.y = rnd();
	return_value.z = rnd();

	if (norm) 
		return return_value.normalize();
	else
		return return_value;
}

vector_struct rnd_vector(bool norm) {
	vector_struct return_value;

	return_value.x = rnd();
	return_value.y = rnd();
	return_value.z = rnd();

	if (norm) 
		return return_value.normalize();
	else
		return return_value;
}

matrix_struct rnd_matrix(bool norm) {
	matrix_struct return_value;

	return_value.x = rnd_vector(norm);
	return_value.y = rnd_vector(norm);
	return_value.z = rnd_vector(norm);

	return return_value;
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

void set_settings() {
	// Set ahrs settings.
	alg.settings.accel_rejection = 0.1;
	alg.settings.accel_rejection_t = 0.1;
	alg.settings.add_declination = false;
	alg.settings.gain_init = 20;
	alg.settings.gain_normal = 10;
	alg.settings.init_time = 3;
}


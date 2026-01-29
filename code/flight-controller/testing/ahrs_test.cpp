#include <iostream>
#include <unistd.h>

#include "../ahrs.h"
#include "gen_data.h"

using namespace std;

static const double dt = 0.0015;

ahrs alg;

// Clamp a value between two values.
double clamp(double value, double min, double max);

void set_settings();

void linear_sim();

void random_sim();

int main() {
	set_settings();

	sim_settings sim_sets;

	double runtime = 3; // Simulation runtime in seconds.

	double timestamp = alg.get_timestamp(1);

	cout << "starting!" << endl;
	while (alg.get_timestamp(1) < timestamp + (runtime * 1000000)) {
		double dt_timestamp = alg.get_timestamp(1);

		imu_data data = gen_data();

		output_struct output = alg.update(data.gyro, data.accel, data.mag, dt);

		// Make dt constant.
		double time_elapsed = alg.get_timestamp(1) - dt_timestamp;

		usleep(clamp(dt * 1000000 - time_elapsed, 0, dt * 1000000));
	}
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
	// Set gen data settings.

	// Set ahrs settings.
	alg.settings.accel_rejection = 0.1;
	alg.settings.accel_rejection_t = 0.1;
	alg.settings.add_declination = false;
	alg.settings.gain_init = 20;
	alg.settings.gain_normal = 10;
	alg.settings.init_time = 3;
}


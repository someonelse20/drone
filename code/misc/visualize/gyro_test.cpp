#include <iostream>
#include <chrono>
#include <thread>
#include "get_data.h"
#include "../ahrs.h"

using namespace std;

vector_struct orientation = {0.0, 0.0, 0.0};

double dt = 0.013;

vector_struct vector_to_struct(vector v) {
	return {v.x, v.y, v.z};
}

vector_struct gyro_function() {
	return vector_to_struct(read_gyro());
}

int main() {
	imu_init();

	ahrs alg;

	vector_struct bias = alg.gyro_bias_calibration(1, gyro_function);

	while (true) {
		double timestamp = alg.get_timestamp();

		vector_struct rate_of_change = vector_to_struct(read_gyro());

		rate_of_change = alg.calibrate_gyro_accel(rate_of_change, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {1, 1, 1}, bias);

		// cout << rate_of_change.x << "," << rate_of_change.y << "," << rate_of_change.z << endl;

		orientation = alg.add_vector(orientation, alg.scale_vector(rate_of_change, dt));

		cout << orientation.x << "," << orientation.y << "," << orientation.z << endl;
		
		// cout << alg.get_timestamp() - timestamp << endl;

		this_thread::sleep_for(chrono::milliseconds(10));
	}
}


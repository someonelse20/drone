#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include "get_data.h"
#include "../ahrs.h"

using namespace std;

quaternion_struct orientation = {1.0, 0.0, 0.0, 0.0};

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

		vector_struct gyro = vector_to_struct(read_gyro());

		gyro = alg.calibrate_gyro_accel(gyro, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {1, 1, 1}, bias);

		gyro = alg.scale_vector(gyro, M_PI / 180);

		/*
		quaternion_struct rate_of_change = alg.quaternion_product(alg.scale_quaternion(orientation, 0.5), {0, gyro.x, gyro.y, gyro.z});

		orientation = alg.add_quaternion(orientation, alg.scale_quaternion(rate_of_change, dt));
		*/

		double theta = alg.vector_norm(gyro) * dt;

		quaternion_struct rate_of_change;

		if (theta < 0.00001) {
			vector_struct gyro_product = alg.scale_vector(gyro, 0.5 * dt);
			rate_of_change = {1, gyro_product.x, gyro_product.y, gyro_product.z};
		}
		else {
			vector_struct gyro_product = alg.scale_vector(alg.scale_vector(gyro, 1/alg.vector_norm(gyro)), sin(theta/2));
			rate_of_change = {cos(theta/2), gyro_product.x, gyro_product.y, gyro_product.z};
		}

		// orientation = alg.quaternion_product(rate_of_change, orientation);
		orientation = alg.quaternion_product(orientation, rate_of_change);

		orientation = alg.normalize_quaternion(orientation);

		vector_struct euler = alg.quaternion_to_euler(orientation);

		// cout << orientation.w << "," << orientation.x << "," << orientation.y << "," << orientation.z << endl;
		cout << euler.x << "," << euler.y << "," << euler.z << endl;
		
		// cout << alg.get_timestamp() - timestamp << endl;

		this_thread::sleep_for(chrono::milliseconds(10));
	}
}


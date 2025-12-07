#include <iostream>
#include <cmath>
#include "get_data.h"
#include "../ahrs.h"

using namespace std;

vector_struct vector_to_struct(vector v) {
	return {v.x, v.y, v.z};
}

int main() {
	imu_init();

	ahrs alg;

	vector_struct bias = {-0.02425592, 0.00482671, 0.00482671};
	vector_struct sensitivity = {1.00154462, 0.99859621, 0.99785103};
	
	double declination = 80;

	while (true) {
		vector_struct accel = vector_to_struct(read_accel());

		accel = alg.calibrate_gyro_accel(accel, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, sensitivity, bias);

		cout << accel.x << ", " << accel.y << ", " << accel.z << endl;
	}
}


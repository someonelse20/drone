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

	vector_struct hard_iorn = {-0.03810718,  0.18329252, -0.13790179};
	matrix_struct soft_iorn = {{1.04397915e+00, 6.24500451e-17, 2.77555756e-17},
                               {6.24500451e-17, 1.04397915e+00, 9.43689571e-16},
                               {0.00000000e+00, 9.43689571e-16, 1.04397915e+00}};

	double declination = 0;

	while (true) {
		vector_struct mag = vector_to_struct(read_mag());

		mag = alg.calibrate_mag(mag, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, soft_iorn, hard_iorn);

		// cout << mag.x << ", " << mag.y << ", " << mag.z << endl;

		double heading = atan2(mag.x, mag.y);

		cout << heading * (180 / M_PI) << endl;

		/*
		double mx = 0.5 * cos(15) * sin(heading);
		double my = 0.5 * cos(15) * cos(heading);

		double norm = sqrt(pow(mx, 2) + pow(my, 2));

		cout << atan2(mx/norm, my/norm) * (180 / M_PI) << endl;
		*/
	}
}


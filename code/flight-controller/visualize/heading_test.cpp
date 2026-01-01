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

	matrix_struct soft_iorn = {{ 0.90782961, -0.05349811,  0.0207239 },
                                                 {-0.11265483, -0.91579853,  0.02113201},
                                                 {-0.06981889, -0.12123726,  0.91697565}};
	vector_struct hard_iorn = {-0.00618111, -0.12120257, -0.15837325};


	double declination = 0;

	while (true) {
		vector_struct mag = vector_to_struct(read_mag());

		mag = alg.calibrate_mag(mag, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, soft_iorn, hard_iorn);

		// cout << mag.x << ", " << mag.y << ", " << mag.z << endl;

		// double heading = atan2(mag.x, mag.y);

		double heading = atan2(mag.y, mag.x);

		cout << heading * (180 / M_PI) << endl;

		/*
		double mx = 0.5 * cos(15) * sin(heading);
		double my = 0.5 * cos(15) * cos(heading);

		double norm = sqrt(pow(mx, 2) + pow(my, 2));

		cout << atan2(mx/norm, my/norm) * (180 / M_PI) << endl;
		*/
	}
}


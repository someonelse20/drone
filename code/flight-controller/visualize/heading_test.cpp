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

	vector_struct hard_iorn = {0.06474413, 0.18897978, -0.16735554};
	matrix_struct soft_iorn = {{-1.11146911, 0.01387134, -0.01505031},
                               {0.13072505, 1.11913704, 0.14382757},
                               {-0.01697672, -0.00898346, 1.10987745}};

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


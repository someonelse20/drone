#include <iostream>
#include "get_data.h"

using namespace std;

int main() {
	cout << "Hello World!" << endl;

	imu_init();

	vector gyro = read_gyro();

	cout << gyro.x << endl;
}


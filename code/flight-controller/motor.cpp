#include <pigpio.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "motor.h"

using namespace std;

int front_left = 16;
int front_right = 19;
int back_left = 20;
int back_right = 21;

motor::motor(int GPIO, int min_speed, int max_speed) {
	;
}

int motor::set_speed(int speed) {
	if (speed > 1000 && speed < 2000) {
		gpioServo(GPIO, (int)round(min_speed + (max_speed - min_speed) * (speed / 1000)));
		return 0;
	}
	else {
		cout << "error, bad pulsewidth: " << speed << endl;
		return -1;
	}
}

void motor::calibrate() {
	cout << "WARNING! TAKE OFF PROPS!" << endl;
	cout << "enter -1 to continue to next step" << endl;

	for (int i = 0; i < 2; i++) {
		int input = 0;
		while (input >= 0) {
			int speed = 1000;

			cout << "enter motor speed: ";
			cin >> input;

			if (input > 0) {
				if (i == 0) {
					min_speed = speed;
					cout << "minimum speed set to " << min_speed << endl;
				}
				else {
					max_speed = speed;
					cout << "maximum speed set to " << max_speed << endl;
				}

				speed = 1000;

				break;
			}
			else {
				speed = input;
			}

			set_speed(speed);
		}
	}

	cout << "min: " << min_speed << " max: " << max_speed << endl;
	cout << "min and max speed temperaraly set, you should make it permanent" << endl;
}

int motor::test_class() {
	cout << GPIO;
	return GPIO;
}

/*
int main (void)
{
	int speed = 0;

	if (gpioInitialise() == -1)
		exit(1);

	while(1)
	{
		gpioServo(front_left, speed);
		gpioServo(back_left, speed);
		gpioServo(back_left, speed);
		gpioServo(back_right, speed);

		cout << "enter motor speed: " << endl;
		cin >> speed;
		cout << "speed set to " << speed << endl;
	}

	return 0;
}
*/


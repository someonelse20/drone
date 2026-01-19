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

motor::motor(int set_GPIO, int set_min_speed, int set_max_speed) {
	GPIO = set_GPIO;
	min_speed = set_min_speed;
	max_speed = set_max_speed;
}

int motor::set_speed(double speed) {
	if (speed >= 0 && speed <= 100) {
		int true_speed = (int)round(min_speed - 1 + (max_speed + 1 - min_speed - 1) * (speed / 100.0));
		gpioServo(GPIO, true_speed);
		return true_speed;
	}
	else {
		cout << "error, bad pulsewidth: " << speed << endl;
		return -1;
	}
}

void motor::stop() {
	gpioServo(GPIO, 1000);
}

void motor::calibrate() {
	cout << "WARNING! TAKE OFF PROPS!" << endl;
	cout << "enter -1 to continue to next step" << endl;

	for (int i = 0; i < 2; i++) {
		int input = 0;
		int speed = 1000;

		while (input >= 0) {
			cout << "enter motor speed: ";
			cin >> input;

			if (input < 0) {
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

			gpioServo(GPIO, speed);
		}
	}

	stop();

	cout << "min: " << min_speed << " max: " << max_speed << endl;
	cout << "min and max speed temperaraly set, you should make it permanent" << endl;
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


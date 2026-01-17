#include <pigpio.h>
#include <cstdlib>
#include <iostream>

using namespace std;

int front_left = 16;
int front_right = 19;
int back_left = 20;
int back_right = 21;

int main (void)
{
	int speed = 1000;

	if (gpioInitialise() == -1)
		exit(1);

	while(1)
	{
		gpioServo(front_left, speed);
		gpioServo(front_right, speed);
		gpioServo(back_left, speed);
		gpioServo(back_right, speed);

		cout << "enter motor speed: " << endl;
		cin >> speed;
		cout << "speed set to " << speed << endl;
	}

	return 0;
}


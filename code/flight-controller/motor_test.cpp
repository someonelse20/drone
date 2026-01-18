#include <iostream>
#include <lgpio.h>

using namespace std;

int front_left = 16;
int front_right = 19;
int back_left = 20;
int back_right = 21;

int main (void)
{
	int h;

	int speed = 0;

	h = lgGpiochipOpen(0);
	if (h == -1)
		exit(1);

	int gpios[4] = {front_left, front_right, back_left, back_right};
	int levels[4] = {0, 0, 0, 0};
	lgGroupClaimOutput(h, 0, 4, gpios, levels);

	while(1)
	{
		lgTxServo(h, front_left, speed, 50, 0, 0);
		lgTxServo(h, front_right, speed, 50, 0, 0);
		lgTxServo(h, back_left, speed, 50, 0, 0);
		lgTxServo(h, back_right, speed, 50, 0, 0);

		cout << "enter motor speed: " << endl;
		cin >> speed;
		cout << "speed set to " << speed << endl;
	}

	return 0;
}


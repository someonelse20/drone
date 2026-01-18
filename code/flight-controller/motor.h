#ifndef MOTOR_H
#define MOTOR_H

class motor {
private:
	int GPIO = 0;
	int min_speed = 1000;
	int max_speed = 2000;

public:
	motor(int GPIO, int min_speed = 1000, int max_speed = 2000);

	int set_speed(int speed); // Speed is a percentage of 0 (off) to 1000 (fully on) between the minimum value and the maximum.

	void calibrate();

	int test_class();
};

#endif

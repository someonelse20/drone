#ifndef MOTOR_H
#define MOTOR_H

class motor {
public:
	int GPIO = 0;
	int min_speed = 1000;
	int max_speed = 2000;

	motor(int set_GPIO, int set_min_speed = 1000, int set_max_speed = 2000);

	int set_speed(double speed); // Speed is a percentage of 0 (off) to 100 (fully on) between the minimum value and the maximum.

	void stop(); // Stops the motor.

	void calibrate();

	void write_config(); // Updates config with motor values.
};

#endif

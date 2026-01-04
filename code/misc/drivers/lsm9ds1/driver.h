#ifndef DRIVER_H
#define DRIVER_H

struct data {
	float gyro[3], accel[3], mag[3];
};

int x, y, z;

void init();

data get_data();

#endif


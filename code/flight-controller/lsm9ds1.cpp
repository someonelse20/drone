#include <cstdint>
#include <iostream>
#include <pigpio.h>
#include <sys/types.h>
#include <uchar.h>
#include "lsm9ds1.h"

using namespace std;

lsm9ds1::lsm9ds1(int addr, int mag_addr, int bus) {
	i2cOpen(bus, addr, 0);
	i2cOpen(bus, mag_addr, 0);
}

imu_data lsm9ds1::read_imu() {
	imu_data data;
	return data;
}

void lsm9ds1::test_imu(){

}

int main() {
	int h;
	unsigned addr = 0x6b;
	unsigned reg = 0x28;
	char buf[2];

	gpioInitialise();

	h = i2cOpen(1, addr, 0);

	i2cWriteByteData(h, 0x22, 0x05);
	i2cWriteByteData(h, 0x10, 0x78);
	i2cWriteByteData(h, 0x20, 0x70);
	cout << i2cReadWordData(h, reg) * 2.0 / 32768.0 << endl;

	int16_t data[3];
	i2cReadI2CBlockData(h, reg, (char*)data, sizeof(data));
}


#include <iostream>
#include <wiringPi.h>
#include <wiringPiSPI.h>

using namespace std;

void send(int channel, int speed, int mode, unsigned char data[3]) {
	cout << wiringPiSPISetupMode(channel, speed, mode) << endl;

	cout << "input data" << endl;
	for (int i = 0; i < 3; i++) {
		cout << int(data[i]) << ", ";
	}

	cout << endl;

	cout << wiringPiSPIDataRW(channel, data, 3) << endl;

	cout << "output data" << endl;
	for (int i = 0; i < 3; i++) {
		cout << int(data[i]) << ", ";
	}

	cout << endl;
	cout << "" << endl;

	wiringPiSPIClose(channel);
}

int main() {
	/*for (int channel = 0; channel <= 1; channel++) {
		for (int speed = 50000; speed <= 7000000; speed = speed + 50000) {
			send(channel, speed, 0b10000000);
			send(channel, speed, 0b00000001);
		} 
	}*/
	for (int i = 0; i <= 3; i++) {
		unsigned char data[3] =  {0x3C, 0xAD, 1};
		send(0, 500000, i, data);
	}

	return 0;
}


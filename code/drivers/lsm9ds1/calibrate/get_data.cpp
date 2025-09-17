#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>

using namespace std;

LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

void record(string filename, int runtime) {
	cout << "running!" << endl;
	ofstream datafile("../calibrate/data/" + filename); 

	time_t timestamp;
	time(&timestamp);
	time_t endtime = timestamp + runtime;
	
	while (timestamp <= endtime) {
        while (!imu.gyroAvailable()) ;
        imu.readGyro();
        while(!imu.accelAvailable()) ;
        imu.readAccel();
        while(!imu.magAvailable()) ;
        imu.readMag();

		datafile << "gyro:" + to_string(imu.calcGyro(imu.gx)) + "," + to_string(imu.calcGyro(imu.gy)) + "," + to_string(imu.calcGyro(imu.gz)) + "\n";
		datafile << "accel:" + to_string(imu.calcAccel(imu.ax)) + "," + to_string(imu.calcAccel(imu.ay)) + "," + to_string(imu.calcAccel(imu.az)) + "\n";
		datafile << "mag:" + to_string(imu.calcMag(imu.mx)) + "," + to_string(imu.calcMag(imu.my)) + "," + to_string(imu.calcMag(imu.mz)) + "\n";
		cout << "gyro:" + to_string(imu.calcGyro(imu.gx)) + "," + to_string(imu.calcGyro(imu.gy)) + "," + to_string(imu.calcGyro(imu.gz)) + "\n";
		cout << "accel:" + to_string(imu.calcAccel(imu.ax)) + "," + to_string(imu.calcAccel(imu.ay)) + "," + to_string(imu.calcAccel(imu.az)) + "\n";
		cout << "mag:" + to_string(imu.calcMag(imu.mx)) + "," + to_string(imu.calcMag(imu.my)) + "," + to_string(imu.calcMag(imu.mz)) + "\n";
        usleep(100000);
		time(&timestamp);
	}

	datafile.close();

	return;
}

int main(int argc, char *argv[]) {
	imu.settings.accel.scale = 4;
	imu.settings.gyro.scale = 500;
    imu.begin();
    if (!imu.begin()) {
        fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        exit(EXIT_FAILURE);
    }

	while(1) {
		string filename;
		int runtime;
		cout << "Enter datafile to record: ";
		cin >> filename;
		cout << "Enter runtime in seconds: ";
		cin >> runtime;
		cout << "starting" << endl;

		record(filename, runtime);
	}
}


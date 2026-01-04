#include <cstdlib>
#include <unistd.h>
#include <LSM9DS1_Types.h>
#include "LSM9DS1.h"
#include "get_data.h"

using namespace std;

LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

// Initialize LSM9DS1.
void imu_init(int accel_scale, int gyro_scale, int mag_scale) { // Set default settings for LSM9DS1. mag_scale is not currently used.
	// Set settings
	imu.settings.gyro.scale = gyro_scale;
	imu.settings.accel.scale = accel_scale;
	//imu.settings.mag.scale = mag_scale;

	imu.begin();
	if (!imu.begin()) {
		exit(EXIT_FAILURE);
	}
	// Possibly remove later.
	// imu.calibrate();
};

// Read data from LSM9DS1 and output the gyro, accel, and mag data in a struct.
vector read_gyro() {
	vector gyro;

	while (!imu.gyroAvailable()) ;
	imu.readGyro();

	gyro.x = imu.calcGyro(imu.gx);
	gyro.y = imu.calcGyro(imu.gy);
	gyro.z = imu.calcGyro(imu.gz);

	return gyro;
}

vector read_accel() {
	vector accel;

	while (!imu.accelAvailable()) ;
	imu.readAccel();

	accel.x = imu.calcAccel(imu.ax);
	accel.y = imu.calcAccel(imu.ay);
	accel.z = imu.calcAccel(imu.az);

	return accel;
}

vector read_mag() {
	vector mag;

	while (!imu.magAvailable()) ;
	imu.readMag();

	mag.x = imu.calcMag(imu.mx);
	mag.y = imu.calcMag(imu.my);
	mag.z = imu.calcMag(imu.mz);

	return mag;
}


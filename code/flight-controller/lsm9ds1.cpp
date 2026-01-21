#include <iostream>
#include <pigpio.h>
#include <string>
#include <bitset>
#include <sys/types.h>
#include "lsm9ds1_reg.h"
#include "lsm9ds1.h"

using namespace std;

lsm9ds1::lsm9ds1(int addr, int mag_addr, int bus) {
	ADDR = addr;
	MAG_ADDR = mag_addr;
	BUS = bus;
}

void lsm9ds1::init() {
	// Open communication with imu.
	handle = i2cOpen(BUS, ADDR, 0);
	m_handle = i2cOpen(BUS, MAG_ADDR, 0);

	// Reset device.
	i2cWriteByteData(handle, CTRL_REG8, 0b00000101); 
	i2cWriteByteData(m_handle, CTRL_REG2_M, 0b00001100);

	// Turn on mag.
	i2cWriteByteData(m_handle, CTRL_REG3_M, 0b00000000);

	// Construct indexes to make settings easier.
	if (settings.gyro_scale == 250) {
		gyro_scale = 0;
	} else if (settings.gyro_scale == 500) {
		gyro_scale = 1;
	} else if (settings.gyro_scale == 2000) {
		gyro_scale = 2;
	} else {
		cout << "Incorrect gyro scale: " << settings.gyro_scale << ", defaulting to 250" << endl;
		gyro_scale = 0;
	}

	if (settings.accel_scale == 2) {
		accel_scale = 0;
	} else if (settings.accel_scale == 4) {
		accel_scale = 1;
	} else if (settings.accel_scale == 8) {
		accel_scale = 2;
	} else if (settings.accel_scale == 16) {
		accel_scale = 3;
	} else {
		cout << "Incorrect accel scale: " << settings.accel_scale << ", defaulting to 2" << endl;
		accel_scale = 0;
	}

	if (settings.mag_scale == 4) {
		mag_scale = 0;
	} else if (settings.mag_scale == 8) {
		mag_scale = 1;
	} else if (settings.mag_scale == 12) {
		mag_scale = 2;
	} else if (settings.mag_scale == 16) {
		mag_scale = 3;
	} else {
		cout << "Incorrect mag scale: " << settings.mag_scale << ", defaulting to 4" << endl;
		mag_scale = 0;
	}

	if (settings.data_rate == 14.9) {
		data_rate = 1;
	} else if (settings.data_rate == 59.5) {
		data_rate = 2;
	} else if (settings.data_rate == 119) {
		data_rate = 3;
	} else if (settings.data_rate == 238) {
		data_rate = 4;
	} else if (settings.data_rate == 476) {
		data_rate = 5;
	} else if (settings.data_rate == 952) {
		data_rate = 6;
	} else {
		cout << "Incorrect data rate: " << settings.data_rate << ", defaulting to 952" << endl;
		data_rate = 6;
	}

	if (settings.mag_data_rate == 0.625) {
		mag_data_rate = 0;
	} else if (settings.mag_data_rate == 1.25) {
		mag_data_rate = 1;
	} else if (settings.mag_data_rate == 2.5) {
		mag_data_rate = 2;
	} else if (settings.mag_data_rate == 5) {
		mag_data_rate = 3;
	} else if (settings.mag_data_rate == 10) {
		mag_data_rate = 4;
	} else if (settings.mag_data_rate == 20) {
		mag_data_rate = 5;
	} else if (settings.mag_data_rate == 40) {
		mag_data_rate = 6;
	} else if (settings.mag_data_rate == 80) {
		mag_data_rate = 7;
	} else {
		cout << "Incorrect mag data rate: " << settings.mag_data_rate << ", defaulting to 80" << endl;
		mag_data_rate = 7;
	}

	// Construct binary string.
	string gyro_settings;
	string accel_settings;
	string mag_settings_1;
	string mag_settings_2 = "0";
	string mag_settings_3 = "0000";

	if (settings.mag_temp_comp) {
		mag_settings_1 = "1";
	} else {
		mag_settings_1 = "0";
	}

	mag_settings_1 += mag_modes[settings.mag_mode];
	mag_settings_3  += mag_modes[settings.mag_mode];

	gyro_settings = data_rates[data_rate];
	accel_settings = data_rates[data_rate];
	mag_settings_1 += mag_data_rates[mag_data_rate];

	gyro_settings += gyro_scales[gyro_scale];
	accel_settings += accel_scales[accel_scale];
	mag_settings_2 += mag_scales[mag_scale]; 

	// Make string 8 bit.
	gyro_settings += "000";
	accel_settings += "000";
	mag_settings_1 += "00";
	mag_settings_2 += "00000";
	mag_settings_3 += "00";

	// Set settings.
	i2cWriteByteData(handle, CTRL_REG1_G, (int)stoi(gyro_settings, nullptr, 2));
	i2cWriteByteData(handle, CTRL_REG6_XL, (int)stoi(accel_settings, nullptr, 2));

	i2cWriteByteData(m_handle, CTRL_REG1_M, (int)stoi(mag_settings_1, nullptr, 2));
	i2cWriteByteData(m_handle, CTRL_REG2_M, (int)stoi(mag_settings_2, nullptr, 2));
	i2cWriteByteData(m_handle, CTRL_REG4_M, (int)stoi(mag_settings_3, nullptr, 2));
}

imu_data lsm9ds1::read() {
	string data_ready = bitset<8>(i2cReadByteData(handle, STATUS_REG_0)).to_string();
	string mag_data_ready = bitset<8>(i2cReadByteData(m_handle, STATUS_REG_M)).to_string();

	// Check if imu is still booting.
	if (!data_ready.compare(4, 1, "1")) {
		return data;
	}

	if (!data_ready.compare(6, 1, "1")) {
		double sensitivity = gyro_sensitivities[gyro_scale] / 1000;
		int16_t gyro[3];
		i2cReadI2CBlockData(handle, OUT_X_L_G, (char*)gyro, sizeof(gyro));

		data.gyro.x = gyro[0] * sensitivity;
		data.gyro.y = gyro[1] * sensitivity;
		data.gyro.z = gyro[2] * sensitivity;
	}
	if (!data_ready.compare(7, 1, "1")) {
		double sensitivity = accel_sensitivities[accel_scale] / 1000;
		int16_t accel[3];
		i2cReadI2CBlockData(handle, OUT_X_L_XL, (char*)accel, sizeof(accel));

		data.accel.x = accel[0] * sensitivity;
		data.accel.y = accel[1] * sensitivity;
		data.accel.z = accel[2] * sensitivity;
	}
	if (!mag_data_ready.compare(4, 1, "1")) {
		double sensitivity = mag_sensitivities[mag_scale] / 1000;
		int16_t mag[3];
		i2cReadI2CBlockData(m_handle, OUT_X_L_M, (char*)mag, sizeof(mag));

		data.mag.x = mag[0] * sensitivity;
		data.mag.y = mag[1] * sensitivity;
		data.mag.z = mag[2] * sensitivity;
	}

	return data;
}

/*
int main() {
	gpioInitialise();

	imu_settings settings;

	settings.gyro_scale = 500;
	settings.accel_scale = 4;
	
	settings.data_rate = 238;

	lsm9ds1 imu(0x6b, 0x1e, 1, settings);

	imu.read();

	int h;
	unsigned addr = 0x6b;
	unsigned reg = 0x28;
	char buf[2];

	gpioInitialise();

	h = i2cOpen(1, addr, 0);

	i2cWriteByteData(h, 0x22, 0b00000101); // Reset device.

	i2cWriteByteData(h, 0x10, 0b11000000); // 952hz data rate and 250dps gyro range.
	i2cWriteByteData(h, 0x20, 0b11010000); // 952hz data rate and 4g accel range.
	// cout << i2cReadByteData(h, 0x17) << endl; // data ready

	int16_t data[3];
	i2cReadI2CBlockData(h, reg, (char*)data, sizeof(data));
	// cout << data[2] * 0.061 / 1000 << endl;
	cout << data[2] * 4 / 32768.0 << endl;
}
*/


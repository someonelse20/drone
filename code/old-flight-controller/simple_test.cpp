#include "Fusion/Fusion/Fusion.h"
#include <stdbool.h>
#include <stdio.h>

#include "driver.h"

#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

int main() {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

	init();

    while (true) { // this loop should repeat each time new gyroscope data is available
		data raw_data = get_data();
        const FusionVector gyroscope = {raw_data.gyro[x], raw_data.gyro[y], raw_data.gyro[z]}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {raw_data.accel[x], raw_data.accel[y], raw_data.accel[z]}; // replace this with actual accelerometer data in g

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    }
}

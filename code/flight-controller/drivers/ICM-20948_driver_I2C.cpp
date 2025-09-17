#include <wiringPi.h>
#include <ICM-20948-reg.h>

#define CLOCK_SPEED 7000000
#define SPI_MODE 0

class ICM42605 {
    // ---------- SPI Variables ----------
    static int SPI_BUS;
    static int SPI_CHANNEL;
    static int INT_PIN;
    
    // ---------- Sensor Variables ----------
    static int ACCEL_SENSE;
    static int GYRO_SENSE;
    static int TEMP_SENSE;
    static int TEMP_OFFSET;

    ICM42605(int spi_bus, int spi_channel, int int_pin);

    struct data {
        int accel_x, accel_y, accel_z;
        int gyro_x, gyro_y, gyro_z;
        int temp;
    };

    public:
        data get_data() {
            unsigned char mesg[15];
            mesg[0] = 0x2D;
            wiringPiSPIxDataRW(SPI_BUS, SPI_CHANNEL, mesg, 15);
            
            data output;
            output.accel_x = mesg[1] + mesg[2] / ACCEL_SENSE; 
            output.accel_y = mesg[3] + mesg[4] / ACCEL_SENSE;
            output.accel_z = mesg[5] + mesg[6] / ACCEL_SENSE;
            output.gyro_x = mesg[7] + mesg[8] / GYRO_SENSE;
            output.gyro_y = mesg[9] + mesg[10] / GYRO_SENSE;
            output.gyro_z = mesg[11] + mesg[12] / GYRO_SENSE;
            output.temp = (((mesg[13] + mesg[14]) - TEMP_OFFSET) / TEMP_SENSE) + 21;

            return output;
        }

        void config() {

        }

        unsigned char who_am_i() {
            unsigned char mesg[2] = {0x00, 0};
            wiringPiSPIxDataRW(SPI_BUS, SPI_CHANNEL, mesg, 2);
            return mesg[1];
        }
};

ICM42605::ICM42605(int spi_bus, int spi_channel, int int_pin) {
    SPI_BUS = spi_bus;
    SPI_CHANNEL = spi_channel;
    INT_PIN = int_pin;

    wiringPiSetupPinType(WPI_PIN_BCM);
    wiringPiSPIxSetupMode(SPI_BUS, SPI_CHANNEL, CLOCK_SPEED, SPI_MODE);
}


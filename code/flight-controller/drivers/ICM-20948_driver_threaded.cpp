#include <iostream>
#include <thread>

using namespace std;

class ICM42605 {
    static int CS_PIN;
    static int INT_PIN;
    static int SCLK_PIN;
    static int SDI_PIN; //SDI from the perspective of the master
    static int SDO_PIN; //SDO from the perspective of the master

    float temp;

    static bool loop;

    struct data {
        float x, y, z;
    } accel, gyro, mag;

    ICM42605(int cs_pin, int int_pin, int sclk_pin, int sdi_pin, int sdo_pin);

    void main() {
        while (loop) {

        };
        // cleanup
    }

    public:
        data get_gyro_raw() {
            return gyro;
        }

        data get_accel_raw() {
            return accel;
        }

        data get_gyro_raw() {
            return gyro;
        }

        float get_temp_raw() {
            return temp;
        }

        void start() {
            loop = true;
            thread main_thread(main);
            main_thread.detach();
        }

        void stop(bool now = false) {
            loop = false;
        }
};

ICM42605::ICM42605(int cs_pin, int int_pin, int sclk_pin, int sdi_pin, int sdo_pin) {
    CS_PIN = cs_pin;
    INT_PIN = int_pin;
    SCLK_PIN = sclk_pin;
    SDI_PIN = sdi_pin;
    SDO_PIN = sdo_pin;

    loop = true;
    thread main_thread(main);
    main_thread.detach();
}

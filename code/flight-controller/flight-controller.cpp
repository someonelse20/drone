#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <cmath>
#include <ctime>
#include <vector>

using namespace std;

static const double DT = 0.01, GRAVITY = 9.81;

// angle variables
vector<double> raw_gyro_value;
vector<double> raw_accel_value;
vector<double> raw_mag_value;
vector<double> filtered_gyro_value;
vector<double> gyro_angle;
vector<double> accel_angle;
vector<double> angle;
vector<double> heading;
vector<double> pid_values;

// motor variables
int front_left_pin;
int front_right_pin;
int back_left_pin;
int back_right_pin;

// input variables
vector<double> set_point = {0, 0, 0};
double throttle;

class pid { // pid controller class
    private:
        double error = 0;
        double previous_error = 0;
        double proportional = 0, integral = 0, derivative = 0;
    public:
        double kP, kI, kD;
        double gain;
        double set_point;
        double procces_value;
        double output;

    pid (double set_kP = 1, double set_kI = 1, double set_kD = 1, double set_gain = 1, double set_set_point = 0, double set_proccess_value = 0) {
        kP = set_kP * set_gain;
        kI = set_kI * set_gain;
        kD = set_kD * set_gain;
        gain = set_gain;
        set_point = set_set_point;
        procces_value = set_proccess_value;
    }

    double loop(double procces_value_overide = INFINITY, double set_point_overide = INFINITY) {
        if (procces_value_overide != INFINITY) {
            procces_value = procces_value_overide;
        }        
        
        if (set_point_overide != INFINITY) {
            set_point = set_point_overide;
        }

        error = set_point - procces_value;

        //cout << "Error = " << error << endl;

        proportional = kP * error;

        //cout << "Proportional = " << proportional << endl;

        //cout << DT << endl;

        integral = integral + (kI * error * DT);
        if (integral == -INFINITY) {integral = 0;}

        //cout << "Integral = " << integral << endl;

        derivative = kD * ((error - previous_error) / DT);

        //cout << "Derivative = " << derivative << endl;

        previous_error = error;

        output = proportional + integral + derivative;

        return output;
    }

    void reset() {
        error = 0;
        integral = 0;
    }
};

class motor {
    public:
        int pin;

        motor(int set_pin) {
            pin = set_pin;
        }

        void write(int value) { // DON'T FORGET TO ADD MIN AND MAX THROTTLE

        }

        void start_pwm() {

        }

        void stop_pwm() {

        }
};

// angle calculation functions
vector<double> get_gyro_filter() {
    return raw_gyro_value; // temporary
}

vector<double> get_gyro_angle() {
    vector<double> output = {
        angle[0] + filtered_gyro_value[0] * DT,
        angle[1] + filtered_gyro_value[1] * DT,
        angle[2] + filtered_gyro_value[2] * DT,
    };

    return output;
}

vector<double> get_accel_angle() { // accelerometers can only calculate roll pitch so this vector only has 2 variables
    vector<double> output = {
        /*180 * atan (raw_accel_value[0]/sqrt(raw_accel_value[1]*raw_accel_value[1] + raw_accel_value[2]*raw_accel_value[2]))/M_PI,
        180 * atan (raw_accel_value[1]/sqrt(raw_accel_value[0]*raw_accel_value[0] + raw_accel_value[2]*raw_accel_value[2]))/M_PI*/
        
    };

    return output;
}

vector<double> get_fused_angle(const double gyro_bias = 98, const double accel_bias = 2) {
    vector<double> output = {
        gyro_bias * gyro_angle[0] + accel_bias * accel_angle[0],
        gyro_bias * gyro_angle[1] + accel_bias * accel_angle[1],
        gyro_bias * gyro_angle[2] + accel_bias * accel_angle[2]
    };
    return output;
}

vector<double> get_heading() {
    return vector<double> {};
}

double constrain(double value, double min, double max) {
    if (value < min) {
        return min;
    }
    else if (value > max) {
        return max;
    }
    else {
        return value;
    }
}

int main()
{
    pid pid_x;
    pid pid_y;
    pid pid_z;

    motor front_left_motor(front_left_pin);
    motor front_right_motor(front_right_pin);
    motor back_left_motor(back_left_pin);
    motor back_right_motor(back_right_pin);

    cout << "Starting loop! Here we go!" << endl;

    while (true) { // main loop
        const clock_t begin_time = clock();

        filtered_gyro_value = get_gyro_filter();

        gyro_angle = get_gyro_angle();
        accel_angle = get_accel_angle();
        angle = get_fused_angle();
        heading = get_heading();

        pid_values = vector<double> {
            pid_x.loop(angle[0], set_point[0]),
            pid_y.loop(angle[1], set_point[1]),
            pid_z.loop(angle[2], set_point[2])
        };

        // write pid values to motors
        front_left_motor.write(throttle + pid_values[0] + pid_values[1] /*- pid_values[2]*/);
        front_right_motor.write(throttle - pid_values[0] + pid_values[1] /*+ pid_values[2]*/);
        back_left_motor.write(throttle + pid_values[0] - pid_values[1] /*+ pid_values[2]*/);
        back_right_motor.write(throttle - pid_values[0] - pid_values[1] /*- pid_values[2]*/);

        // make DT constant
        double difference = double(clock() - begin_time) / CLOCKS_PER_SEC;
        double delay = DT - difference;
        sleep(delay);
    }

    return 0;
}

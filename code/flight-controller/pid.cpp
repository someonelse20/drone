#include "pid.h"

pid_controller::pid_controller (double set_DT, double set_kP, double set_kI, double set_kD, double set_gain, double set_set_point, double set_proccess_value) {
	DT = set_DT;
	kP = set_kP;
	kI = set_kI;
	kD = set_kD;
	gain = set_gain;
	set_point = set_set_point;
	procces_value = set_proccess_value;
}

double pid_controller::loop(double procces_value_overide, double set_point_overide) {
	if (procces_value_overide != INFINITY) {
		procces_value = procces_value_overide;
	}

	if (set_point_overide != INFINITY) {
		set_point = set_point_overide;
	}

	error = set_point - procces_value;

	proportional = kP * error;

	integral = integral + (kI * error * DT);
	if (integral == -INFINITY) 
		integral = 0;

	derivative = kD * ((error - previous_error) / DT);

	previous_error = error;

	output = proportional + integral + derivative;

	return output * gain;
}

void pid_controller::reset() {
	error = 0;
	integral = 0;
}


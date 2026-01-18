#include <cmath>
#include <set>

class pid_controller { // pid controller class
private:
	double error = 0;
	double previous_error = 0;
	double proportional = 0, integral = 0, derivative = 0;

public:
	double DT;
	double kP, kI, kD;
	double gain;
	double set_point;
	double procces_value;
	double output;

	pid_controller (double set_DT, double set_kP = 1, double set_kI = 1, double set_kD = 1, double set_gain = 1, double set_set_point = 0, double set_proccess_value = 0);

	double loop(double procces_value_overide = INFINITY, double set_point_overide = INFINITY);

	void reset();
};


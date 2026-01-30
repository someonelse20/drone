#include "../ahrs/ahrs.h"

static const double dt = 0.0015;

struct imu_data {
	vector_struct gyro;
	vector_struct accel;
	vector_struct mag;
};

struct sim_settings {
	double runtime = 3;
};

void test_operators(ahrs &alg, sim_settings settings);

void linear_sim(ahrs &alg, sim_settings settings);

void random_sim(ahrs &alg, sim_settings settings);

// Generates random number between 0 and 1.
double rnd();

// Generates random quaternion.
quaternion_struct rnd_quaternion(bool norm=true);

// Generates random vector.
vector_struct rnd_vector(bool norm=true);

// Generates random matrix.
matrix_struct rnd_matrix(bool norm=true);

// Clamp a value between two values.
double clamp(double value, double min, double max);

void set_settings();


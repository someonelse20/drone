#ifndef AHRS_H
#define AHRS_H

#include <string>

struct quaternion_struct;
struct vector_struct;

struct quaternion_struct {
	double w, x, y, z;

	quaternion_struct operator+(const quaternion_struct &q);
	quaternion_struct operator-(const quaternion_struct &q);
	quaternion_struct operator*(const quaternion_struct &q);
	quaternion_struct operator*(const double &s);
	bool operator==(const quaternion_struct&q);
	bool operator!=(const quaternion_struct&q);

	quaternion_struct conjugate();

	double norm();
	quaternion_struct normalize();
	vector_struct to_euler();
	std::string to_string(const std::string seperator=" ");
};

struct vector_struct {
	double x, y, z;

	vector_struct operator+(const vector_struct &v);
	vector_struct operator-(const vector_struct &v);
	vector_struct operator*(const vector_struct &v);
	vector_struct operator*(const double &s);
	bool operator==(const vector_struct &v);
	bool operator!=(const vector_struct &v);

	double norm();
	vector_struct normalize();
	quaternion_struct to_quaternion();
	std::string to_string(const std::string seperator=" ");
};

struct matrix_struct {
	vector_struct x;
	vector_struct y;
	vector_struct z;

	matrix_struct operator*(const matrix_struct &m);
	vector_struct operator*(const vector_struct &v);
	bool operator==(const matrix_struct &m);
	bool operator!=(const matrix_struct &m);

	std::string to_string(const std::string seperator="\n", const std::string vector_seperator=" ");
};

struct orientation_struct {
	quaternion_struct quaterion;
	vector_struct euler;
};

struct acceleration_struct {
	vector_struct zero;
	vector_struct global;
};

struct output_struct {
	orientation_struct orientation; // Orientation of the IMU relative to the earth.
	orientation_struct orientation_earth_frame; // Orientation of the earth relative to the IMU, the default output of the algorithm.
	acceleration_struct acceleration;
	bool accel_rejected = false;
	bool mag_rejected = false;
	bool initialized = false;
};

struct calibrate {
	matrix_struct rotation_matrix = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
	vector_struct sensitivity = {1.0, 1.0, 1.0};
	vector_struct bias = {0.0, 0.0, 0.0};
	matrix_struct soft_iorn = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
	vector_struct hard_iorn = {0.0, 0.0, 0.0};
};

struct settings_struct{
	double gain_normal = 0.5;
	double gain_init = 10.0;
	double init_time = 3.0; // Time the algorithm is initializing in seconds.
	
	double min_mag_distortion = 0.22;
	double max_mag_distoriton = 0.67;
	double declination = 15.2;
	bool add_declination = true;

	double accel_rejection=0.1; // Acceleration in g that the accelerometer should be considered unreliable.
	double accel_rejection_t=0.1; // Time in secconds that acceleration > accel_rejection after which the accelerometer will be rejected.
	
	calibrate gyro_calibrate;
	calibrate accel_calibrate;
	calibrate mag_calibrate;
}; 

class ahrs {
private:
	// ---------------------------------------- VARIABLES ----------------------------------------

	// Orientation as a quaterion. Starts at no rotation.
	quaternion_struct orientation = {1.0, 0.0, 0.0, 0.0}; 

	// Total error of the algorithm.
	vector_struct error = {0.0, 0.0, 0.0};

	// Error of the gyrometer from accelerometer.
	vector_struct a_error = {0.0, 0.0, 0.0};

	// Error of the gyrometer from the magnetometer. 
	vector_struct m_error = {0.0, 0.0, 0.0};

	// Gain of the algorithm, changes as the algorithm initializes.
	double gain;

	// The timestamp when the algorithm starts.
	double start_timestamp = -1;

	// Amount of time the accelerometer is unreliable in seconds.
	double accel_t;

	// Timestamp used for calculating accel_t.
	double accel_t_timestamp = 0.0;

public:
	settings_struct settings;

	// ---------------------------------------- FUNCTIONS ----------------------------------------

	// Calculates normal of an euler angle.
	double vector_norm(vector_struct e);

	// Calculates normal of a quaternion.
	double quaternion_norm(quaternion_struct q);

	// Converts quaterion to euler angle in degrees.
	vector_struct quaternion_to_euler(quaternion_struct q);

	// Converts euler angle in radians to quaternion.
	quaternion_struct euler_to_quaternion(vector_struct v);

	// Normalizes quaternion.
	quaternion_struct normalize_quaternion(quaternion_struct q);

	// Calculates conjugate of quaterion.
	quaternion_struct quaternion_conjugate(quaternion_struct q);

	// Calculates product of two quaternions.
	quaternion_struct quaternion_product(quaternion_struct qa, quaternion_struct qb);

	// Multiplies quaternion by scalar.
	quaternion_struct scale_quaternion(quaternion_struct v, double scalar);

	// Adds two quaternions.
	quaternion_struct add_quaternion(quaternion_struct va, quaternion_struct vb);

	// Subtracts two quaternions.
	quaternion_struct subtract_quaternion(quaternion_struct va, quaternion_struct vb);

	// Normalizes vector.
	vector_struct normalize_vector(vector_struct v);

	// Calcuates cross product of two vectors.
	vector_struct cross_product(vector_struct va, vector_struct vb);

	// Multiplies vector by scalar.
	vector_struct scale_vector(vector_struct v, double scalar);

	// Adds two vectors.
	vector_struct add_vector(vector_struct va, vector_struct vb);

	// Subtracts two vectors.
	vector_struct subtract_vector(vector_struct va, vector_struct vb);

	// Calculates the product of a matrix and a vector.
	vector_struct matrix_vector_product(matrix_struct m, vector_struct v);

	// Calculates the product of two matrices.
	matrix_struct matrix_product(matrix_struct ma, matrix_struct mb);

	// Gets time in seconds since epoch. Change the divider to 1 to get microseconds.
	double get_timestamp(double divider = 1000000.0);

	// Gets current time by subtracting time since epoch by the time the algorithm started.
	double current_time();

	// Low pass filter for gyroscope.
	vector_struct low_pass_filter(vector_struct v);

	// Ignores magnetometer readings when the magnetometer variation is too high and unreliable.
	vector_struct mag_rejection(vector_struct mag, bool* mag_rejected);

	// Ignores accelerometer readings when accelerating too much and the accelerometer is unreliable.
	vector_struct accel_rejection(vector_struct accel, bool* accel_rejected);

	// Calibrates the gyrometer and the accelerometer based off pre-calculated values.
	vector_struct calibrate_gyro_accel(vector_struct value, matrix_struct alignment, vector_struct sensitivity, vector_struct bias);

	// Calibrates the magnetometer based off pre-calculated values.
	vector_struct calibrate_mag(vector_struct mag, matrix_struct alignment, matrix_struct soft_iorn, vector_struct hard_iorn);

	// Gets bias calibration with a set amout of time the system is stationary.
	vector_struct gyro_bias_calibration(double time, vector_struct (*gyro_function)());

	output_struct update(vector_struct gyro, vector_struct accel, vector_struct, double dt);
};

#endif

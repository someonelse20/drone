#include <vector>

/*
#define X = 0
#define Y = 1
#define Z = 2
#define W_Q = 0
#define X_Q = 1
#define Y_Q = 2
#define Z_Q = 3
*/

struct quaternion_struct {
	double w, x, y, z;
};

struct vector_struct {
	double x, y, z;
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
};

struct calibrate {
	std::vector<std::vector<double>> rotation_matrix;
	std::vector<double> sensitivity;
	std::vector<double> bias;
	std::vector<double> soft_iorn;
	std::vector<double> hard_iorn;
};

struct settings_struct{
	double gain_normal=0.5;
	double gain_init=10;
	double init_time=300; // Time the algorithm is initializing in miliseconds.
	
	double min_mag_distortion=0.22;
	double max_mag_distoriton=0.67;
	
	double accel_rejection=0.1; // Acceleration in g that the accelerometer should be considered unreliable.
	double accel_rejection_t=100; // Time in ms that acceleration > accel_rejection after which the accelerometer will be rejected.
	
	calibrate gyro_calibrate;
	calibrate accel_calibrate;
	calibrate mag_calibrate;
}; 

class ahrs {
	private:

	// ---------------------------------------- CONSTANTS ----------------------------------------

	/*
	#define X = 0
	#define Y = 1
	#define Z = 2
	#define W_Q = 0
	#define X_Q = 1
	#define Y_Q = 2
	#define Z_Q = 3
	*/

	// ---------------------------------------- VARIABLES ----------------------------------------

	// Orientation as a quaterion. Starts at no rotation.
	std::vector<double> orientation = {1, 0, 0, 0}; 

	// Total error of the algorithm.
	std::vector<double> error;

	// Error of the gyrometer from accelerometer.
	std::vector<double> a_error;

	// Error of the gyrometer from the magnetometer. 
	std::vector<double> m_error;

	// Gain of the algorithm, changes as the algorithm initializes.
	double gain;

	// The timestamp when the algorithm starts.
	double start_timestamp = 0;

	// Amount of time the accelerometer is unreliable in miliseconds.
	double accel_t;

	// Timestamp used for calculating accel_t.
	double accel_t_timestamp = 0;

	public:

	// ---------------------------------------- STRUCTS ----------------------------------------

	/*
	struct quaternion_struct {
		double w, x, y, z;
	};

	struct vector_struct {
		double x, y, z;
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
	};

	struct calibrate {
		double rotation_matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
		double sensitivity[3] = {0, 0, 0};
		double soft_iorn[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
		double hard_iorn[3] = {0, 0, 0};
	};

	struct settings_struct{
		double gain_normal=0.5;
		double gain_init=10;
		double init_time=300; // Time the algorithm is initializing in miliseconds.

		double min_mag_distortion=0.22;
		double max_mag_distoriton=0.67;

		double accel_rejection=0.1; // Acceleration in g that the accelerometer should be considered unreliable.
		double accel_rejection_t=100; // Time in ms that acceleration > accel_rejection after which the accelerometer will be rejected.

		calibrate gyro_calibrate;
		calibrate accel_calibrate;
		calibrate mag_calibrate;
	};
	*/

	settings_struct settings;

	// ---------------------------------------- FUNCTIONS ----------------------------------------

	// Calculates normal of array.
	double norm(std::vector<double> v);

	// Converts quaterion angle to euler angle.
	std::vector<double> quaternion_to_euler(std::vector<double> q);

	// Calculates conjugate of quaterion.
	std::vector<double> quaternion_conjugate(std::vector<double> q);

	// Calculates product of two quaternions.
	std::vector<double> quaternion_product(std::vector<double> qa, std::vector<double> qb);

	// Calcuates cross product of two vectors.
	std::vector<double> cross_product(std::vector<double> va, std::vector<double> vb);

	// Multiplies vector by scalar.
	std::vector<double> scale_vector(std::vector<double> v, double scalar);

	// Adds two vectors.
	std::vector<double> add_vector(std::vector<double> va, std::vector<double> vb);

	// Subtracts two vectors.
	std::vector<double> subtract_vector(std::vector<double> va, std::vector<double> vb);

	// Gets time in miliseconds since epoch.
	double get_timestamp();

	// Gets current time by subtracting time since epoch by the time the algorithm started.
	double current_time();

	// Currently returns input. Will eventualy replace with some sort of low pass filter.
	std::vector<double> gyro_bias_compensation(std::vector<double> gyro);

	// Ignores magnetometer readings when the magnetometer variation is too high and unreliable.
	std::vector<double> mag_rejection(std::vector<double> mag);

	// Ignores accelerometer readings when accelerating too much and the accelerometer is unreliable.
	std::vector<double> accel_rejection(std::vector<double> accel);

	output_struct update(std::vector<double> gyro, std::vector<double> accel, std::vector<double> mag, double dt);

	// ahrs(settings_struct settings_to_set);
};

/*
void set_settings(settings_struct new_settings);

output_struct update(double gyro[3], double accel[3], double mag[3], double sample_period);
*/


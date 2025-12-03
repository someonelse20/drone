// #import <vector>

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

struct matrix_struct {
	vector_struct x;
	vector_struct y;
	vector_struct z;
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
	matrix_struct rotation_matrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	vector_struct sensitivity = {0, 0, 0};
	vector_struct bias = {0, 0, 0};
	matrix_struct soft_iorn = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	vector_struct hard_iorn = {0, 0, 0};
};

struct settings_struct{
	double gain_normal=0.5;
	double gain_init=10.0;
	double init_time=3.0; // Time the algorithm is initializing in seconds.
	
	double min_mag_distortion=0.22;
	double max_mag_distoriton=0.67;
	
	double accel_rejection=0.1; // Acceleration in g that the accelerometer should be considered unreliable.
	double accel_rejection_t=1000; // Time in ms that acceleration > accel_rejection after which the accelerometer will be rejected.
	
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
	quaternion_struct orientation = {1, 0, 0, 0}; 

	// Total error of the algorithm.
	vector_struct error = {0, 0, 0};

	// Error of the gyrometer from accelerometer.
	vector_struct a_error = {0, 0, 0};

	// Error of the gyrometer from the magnetometer. 
	vector_struct m_error = {0, 0, 0};

	// Gain of the algorithm, changes as the algorithm initializes.
	double gain;

	// The timestamp when the algorithm starts.
	double start_timestamp = -1;

	// Amount of time the accelerometer is unreliable in seconds.
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
	// double norm(std::vector<double> v);

	// Calculates normal of an euler angle.
	double vector_norm(vector_struct e);

	// Calculates normal of a quaternion.
	double quaternion_norm(quaternion_struct q);

	// Converts quaterion angle to euler angle.
	vector_struct quaternion_to_euler(quaternion_struct q);

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

	// Gets time in seconds since epoch.
	double get_timestamp();

	// Gets current time by subtracting time since epoch by the time the algorithm started.
	double current_time();

	// Currently returns input. Will eventualy replace with some sort of low pass filter.
	vector_struct gyro_bias_compensation(vector_struct gyro);

	// Ignores magnetometer readings when the magnetometer variation is too high and unreliable.
	vector_struct mag_rejection(vector_struct mag);

	// Ignores accelerometer readings when accelerating too much and the accelerometer is unreliable.
	vector_struct accel_rejection(vector_struct accel);

	// Calibrates the gyrometer and the accelerometer based off pre-calculated values.
	vector_struct calibrate_gyro_accel(vector_struct value, matrix_struct alignment, vector_struct sensitivity, vector_struct bias);

	// Calibrates the magnetometer based off pre-calculated values.
	vector_struct calibrate_mag(vector_struct mag, matrix_struct soft_iorn, vector_struct hard_iorn);

	output_struct update(vector_struct gyro, vector_struct accel, vector_struct, double dt);

	// ahrs(settings_struct settings_to_set);
};

/*
void set_settings(settings_struct new_settings);

output_struct update(double gyro[3], double accel[3], double mag[3], double sample_period);
*/


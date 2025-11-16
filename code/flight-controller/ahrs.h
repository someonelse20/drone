#include <vector>
#define X = 0
#define Y = 1
#define Z = 2
#define W_Q = 0
#define X_Q = 1
#define Y_Q = 2
#define Z_Q = 3

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
	double rotation_matrix[3][3];
	double sensitivity[3];
	double bias[3];
	double soft_iorn[3][3];
	double hard_iorn[3];
};

struct settings_struct{
	double gain_normal=0.5;
	double gain_init=10;
	double init_time=3;
	
	double min_mag_distortion=0.22;
	double max_mag_distoriton=0.67;
	
	double accel_rejection=0.1; // Acceleration in g that the accelerometer should be considered unreliable.
	double accel_rejection_t=100; // Time in ms that acceleration > accel_rejection after which the accelerometer will be rejected.
	
	calibrate gyro_calibrate;
	calibrate accel_calibrate;
	calibrate mag_calibrate;
};

void set_settings(settings_struct new_settings);

output_struct update(double gyro[3], double accel[3], double mag[3], double sample_period);


#define X = 0
#define Y = 1
#define Z = 2
#define W_Q = 0
#define X_Q = 1
#define Y_Q = 2
#define Z_Q = 3

struct output_quaternion {
	double w, x, y, z;
};
struct output_vector {
	double x, y, z;
};
struct output_orientation {
	output_quaternion quaterion;
	output_vector euler;
};
struct output_acceleration {
	output_vector zero;
	output_vector global;
};
struct outputs {
	output_orientation orientation; // Orientation of the IMU relative to the earth.
	output_orientation orientation_earth_frame; // Orientation of the earth relative to the IMU, the default output of the algorithm.
	output_acceleration acceleration;
};

struct settings_struct{
	double gain_normal=0.5;
	double gain_init=10;
	double init_time=3;
	
	double min_mag_distortion=0.22;
	double max_mag_distoriton=0.67;
	
	double accel_rejection=0.1; // Acceleration in g that the accelerometer should be considered unreliable.
	double accel_rejection_t=100; // Time in ms that acceleration > accel_rejection after which the accelerometer will be rejected.
};

void init(); // Initialises the algorithm.

outputs update(double gyro[3], double accel[3], double mag[3], double sample_period);


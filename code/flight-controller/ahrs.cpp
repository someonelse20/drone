#include <chrono>
#include <cmath>

#include "ahrs.h"

using namespace std;

double ahrs::vector_norm(vector_struct e) {
	return sqrt(pow(e.x, 2) + pow(e.y, 2) + pow(e.z, 2));
}

double ahrs::quaternion_norm(quaternion_struct q) {
	return sqrt(pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2) + pow(q.w, 2));
}

vector_struct ahrs::quaternion_to_euler(quaternion_struct q) {
	vector_struct return_value;

	return_value.x = atan2(2 * (q.y * q.z - q.w * q.x), 2 * pow(q.w, 2) - 1 + 2 * pow(q.z, 2)) * (180.0 / M_PI),
	return_value.y = -asin(2 * (q.x * q.z + q.w * q.y)) * 180.0 / M_PI,
	return_value.z = atan2(2 * (q.x * q.y - q.w * q.z), 2 * pow(q.w, 2) - 1 + 2 * pow(q.x, 2)) * (180.0 / M_PI);

	return return_value;
}

quaternion_struct ahrs::euler_to_quaternion(vector_struct v) {
	quaternion_struct return_value;

	// abbreviations for the various angular functions
	double cr = cos(v.x * 0.5);
	double sr = sin(v.x * 0.5);
	double cp = cos(v.y * 0.5);
	double sp = sin(v.y * 0.5);
	double cy = cos(v.z * 0.5);
	double sy = sin(v.z * 0.5);

	return_value.w = cr * cp * cy + sr * sp * sy;
	return_value.x = sr * cp * cy - cr * sp * sy;
	return_value.y = cr * sp * cy + sr * cp * sy;
	return_value.z = cr * cp * sy - sr * sp * cy;

	return return_value;
}

quaternion_struct ahrs::normalize_quaternion(quaternion_struct q) {
	return scale_quaternion(q, 1 / quaternion_norm(q));
}

quaternion_struct ahrs::quaternion_conjugate(quaternion_struct q) {
	quaternion_struct return_value;

	return_value.w = q.w;
	return_value.x = -q.x;
	return_value.y = -q.y;
	return_value.z = -q.z;

	return return_value;
}

quaternion_struct ahrs::quaternion_product(quaternion_struct qa, quaternion_struct qb) {
	quaternion_struct return_value;

	return_value.w = qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z;
	return_value.x = qa.w * qb.x + qa.x * qb.w + qa.y * qb.z - qa.z * qb.y;
	return_value.y = qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x;
	return_value.z = qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w;

	return return_value;
}

quaternion_struct ahrs::scale_quaternion(quaternion_struct v, double scalar) {
	quaternion_struct return_value;

	return_value.w = v.w * scalar;
	return_value.x = v.x * scalar;
	return_value.y = v.y * scalar;
	return_value.z = v.z * scalar;

	return return_value;
}

quaternion_struct ahrs::add_quaternion(quaternion_struct va, quaternion_struct vb) {
	quaternion_struct return_value;

	return_value.w = va.w + vb.w;
	return_value.x = va.x + vb.x;
	return_value.y = va.y + vb.y;
	return_value.z = va.z + vb.z;

	return return_value;
}

quaternion_struct ahrs::subtract_quaternion(quaternion_struct va, quaternion_struct vb) {
	quaternion_struct return_value;

	return_value.w = va.w - vb.w;
	return_value.x = va.x - vb.x;
	return_value.y = va.y - vb.y;
	return_value.z = va.z - vb.z;

	return return_value;
}

vector_struct ahrs::normalize_vector(vector_struct v) {
	return scale_vector(v, 1 / vector_norm(v));
}

vector_struct ahrs::cross_product(vector_struct va, vector_struct vb) {
	vector_struct return_value;

	return_value.x = va.y * vb.z - va.z * vb.y;
	return_value.y = va.z * vb.x - va.x * vb.z;
	return_value.z = va.x * vb.y - va.y * vb.x;

	return return_value;
}

vector_struct ahrs::scale_vector(vector_struct v, double scalar) {
	vector_struct return_value;

	return_value.x = v.x * scalar;
	return_value.y = v.y * scalar;
	return_value.z = v.z * scalar;

	return return_value;
}

vector_struct ahrs::add_vector(vector_struct va, vector_struct vb) {
	vector_struct return_value;

	return_value.x = va.x + vb.x;
	return_value.y = va.y + vb.y;
	return_value.z = va.z + vb.z;

	return return_value;
}

vector_struct ahrs::subtract_vector(vector_struct va, vector_struct vb) {
	vector_struct return_value;

	return_value.x = va.x - vb.x;
	return_value.y = va.y - vb.y;
	return_value.z = va.z - vb.z;

	return return_value;
}

vector_struct ahrs::matrix_vector_product(matrix_struct m, vector_struct v) {
	vector_struct return_value;

	// return_value.x = v.x * m.x.x + v.y * m.x.y + v.z * m.x.z;
	return_value.x = m.x.x * v.x + m.x.y * v.y + m.x.z * v.z;
	return_value.y = m.y.x * v.x + m.y.y * v.y + m.y.z * v.z;
	return_value.z = m.z.x * v.x + m.z.y * v.y + m.z.z * v.z;

	return return_value;
}

matrix_struct ahrs::matrix_product(matrix_struct ma, matrix_struct mb) {
	matrix_struct return_value;

	return_value.x.x = ma.x.x * mb.x.x + ma.x.y * mb.y.x + ma.x.z * mb.z.x;
	return_value.x.y = ma.x.x * mb.x.y + ma.x.y * mb.y.y + ma.x.z * mb.z.y;
	return_value.x.z = ma.x.x * mb.x.z + ma.x.y * mb.y.z + ma.x.z * mb.z.z;

	return_value.y.x = ma.y.x * mb.x.x + ma.y.y * mb.y.x + ma.y.z * mb.z.x;
	return_value.y.y = ma.y.x * mb.x.y + ma.y.y * mb.y.y + ma.y.z * mb.z.y;
	return_value.y.z = ma.y.x * mb.x.z + ma.y.y * mb.y.z + ma.y.z * mb.z.z;

	return_value.z.x = ma.z.x * mb.x.x + ma.z.y * mb.y.x + ma.z.z * mb.z.x;
	return_value.z.y = ma.z.x * mb.x.y + ma.z.y * mb.y.y + ma.z.z * mb.z.y;
	return_value.z.z = ma.z.x * mb.x.z + ma.z.y * mb.y.z + ma.z.z * mb.z.z;

	return return_value;
}

inline double ahrs::get_timestamp(double divider) {
	using namespace std::chrono;
	return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count() / divider;
}

inline double ahrs::current_time() {
	return get_timestamp() - start_timestamp;
}

vector_struct ahrs::low_pass_filter(vector_struct v) { // Add low pass filter.
	return v;
}

vector_struct ahrs::mag_rejection(vector_struct mag, bool* mag_rejected) {
	double mag_distortion = vector_norm(mag);

	if (mag_distortion > settings.min_mag_distortion && mag_distortion < settings.max_mag_distoriton) {
		*mag_rejected = false;
		return mag;
	}
	else {
		vector_struct return_value;

		*mag_rejected = true;

		return_value.x = 0;
		return_value.y = 0;
		return_value.z = 0;

		return return_value;
	}
}

vector_struct ahrs::accel_rejection(vector_struct accel, bool* accel_rejected){ 
	// calculate the amount of time that the accelerometer measurment is unreliable
	if (accel_t_timestamp == 0)
		accel_t_timestamp = get_timestamp();
	if ((vector_norm(accel) - 1) > -settings.accel_rejection && (vector_norm(accel) - 1) < settings.accel_rejection) 
		accel_t = 0;
	else 
		accel_t += get_timestamp() - accel_t_timestamp;

	if (accel_t > settings.accel_rejection_t) {
		vector_struct return_value;

		*accel_rejected = true;
		return_value.x = 0;
		return_value.y = 0;
		return_value.z = 0;

		return return_value;
	}
	else {
		*accel_rejected = false;
		return accel;
	}
}

vector_struct ahrs::calibrate_gyro_accel(vector_struct value, matrix_struct alignment, vector_struct sensitivity, vector_struct bias) {
	vector_struct calibrated;

	matrix_struct sensitivity_inverse = {{1/sensitivity.x, 0, 0},
                                         {0, 1/sensitivity.y, 0},
                                         {0, 0, 1/sensitivity.z}};

	calibrated = subtract_vector(value, bias);
	calibrated = matrix_vector_product(sensitivity_inverse, calibrated);
	calibrated = matrix_vector_product(alignment, calibrated);

	return calibrated;
}

vector_struct ahrs::calibrate_mag(vector_struct mag, matrix_struct alignment, matrix_struct soft_iorn, vector_struct hard_iorn) {
	vector_struct calibrated;

	calibrated = matrix_vector_product(soft_iorn, mag);
	calibrated = subtract_vector(calibrated, hard_iorn);
	calibrated = matrix_vector_product(alignment, calibrated);

	return calibrated;
}

vector_struct ahrs::gyro_bias_calibration(double time, vector_struct (*gyro_function)()) {
	double timestamp = get_timestamp();

	double x_avg = 0.0, y_avg = 0.0, z_avg = 0.0;
	int len = 0;

	while (get_timestamp() <= timestamp + time) {
		vector_struct gyro_data = gyro_function();

		x_avg += gyro_data.x;
		y_avg += gyro_data.y;
		z_avg += gyro_data.z;

		len++;
	}

	return {x_avg / len, y_avg / len, z_avg / len};
}

output_struct ahrs::update(vector_struct gyro, vector_struct accel, vector_struct mag, double dt) {
	output_struct return_outputs;

	// set timestamp
	if (start_timestamp == -1) {
		start_timestamp = get_timestamp();
	}

	// initialise 
	return_outputs.initialized = current_time() > settings.init_time;

	if (!return_outputs.initialized) 
		gain = settings.gain_normal + (pow(settings.init_time, -current_time()) / settings.init_time) * (settings.gain_init - settings.gain_normal);
	else
		gain = settings.gain_normal;

	// calibrate sensors 
	vector_struct gyro_calibrated = calibrate_gyro_accel(gyro, settings.gyro_calibrate.rotation_matrix, settings.gyro_calibrate.sensitivity, settings.gyro_calibrate.bias);
	vector_struct accel_calibrated = calibrate_gyro_accel(accel, settings.accel_calibrate.rotation_matrix, settings.accel_calibrate.sensitivity, settings.accel_calibrate.bias);
	vector_struct mag_calibrated = calibrate_mag(mag, settings.mag_calibrate.rotation_matrix, settings.mag_calibrate.soft_iorn, settings.mag_calibrate.hard_iorn);

	// sensor conditioning
	vector_struct gyro_conditioned = low_pass_filter(gyro_calibrated);
	vector_struct accel_conditioned = accel_rejection(accel_calibrated, &return_outputs.accel_rejected);
	vector_struct mag_conditioned = mag_rejection(mag_calibrated, &return_outputs.mag_rejected);

	// converts gyro to rad/s from deg/s after conditioning
	gyro_conditioned = scale_vector(gyro_conditioned, M_PI / 180);

	// error calculation
	vector_struct accel_normalized = normalize_vector(accel_conditioned);
	a_error = cross_product(accel_normalized, {(2 * orientation.x * orientation.z) - (2 * orientation.w * orientation.y),
                                               (2 * orientation.y * orientation.z) + (2 * orientation.w * orientation.x),
                                               (2 * pow(orientation.w, 2)) - 1 + (2 * pow(orientation.z, 2))});


	vector_struct cross_a_m = cross_product(accel_normalized, normalize_vector(mag_calibrated));
	m_error = cross_product(cross_a_m, {(-2 * pow(orientation.w, 2)) + 1 - (2 * pow(orientation.x, 2)),
                                        (-2 * orientation.x * orientation.y) + (2 * orientation.w * orientation.z),
                                        (-2 * orientation.x * orientation.z) - (2 * orientation.w * orientation.y)});

	if (vector_norm(accel_conditioned) > 0 && vector_norm(mag_conditioned) > 0)
		error = add_vector(a_error, m_error);
	else if (vector_norm(accel_conditioned) > 0)
		error = a_error;
	else 
		error = {0, 0, 0};

	// complementary filter
	vector_struct gyro_error_product = subtract_vector(gyro_conditioned, scale_vector(error, gain));

	quaternion_struct orientation_rate_of_chage = quaternion_product(scale_quaternion(orientation, 0.5), {0, gyro_error_product.x, gyro_error_product.y, gyro_error_product.z});

	quaternion_struct orientation_unnormalised = add_quaternion(orientation, scale_quaternion(orientation_rate_of_chage, dt));
	orientation = normalize_quaternion(orientation_unnormalised);

	// zero g acceleration calculation (Don't use the rejected acceleration results here.)
	// NOTE: The original paper says to subtract these vectors but it seems to do the opposite, so I added them instead and it works.
	vector_struct acceleration_zero = add_vector(accel_calibrated, {2 * orientation.x * orientation.z - 2 * orientation.w * orientation.y,
	                                                                     2 * orientation.y * orientation.z + 2 * orientation.w * orientation.x,
	                                                                     2 * pow(orientation.w, 2) - 1 + 2 * pow(orientation.z, 2)});

		quaternion_struct orientation_global = quaternion_conjugate(orientation);
	quaternion_struct acceleration_global_quaternion = quaternion_product(quaternion_product(orientation, {0, acceleration_zero.x, acceleration_zero.y, acceleration_zero.z}),
	                                                                   orientation_global);
	vector_struct acceleration_global = {acceleration_global_quaternion.x, acceleration_global_quaternion.y, acceleration_global_quaternion.z};

	// adding declination
	quaternion_struct orientation_declination = orientation;

	if (settings.add_declination) {
		quaternion_struct declination = euler_to_quaternion({0, 0, settings.declination * (M_PI / 180)});
		orientation_declination = quaternion_product(orientation, declination);
	}

	quaternion_struct orientation_global_declination = quaternion_conjugate(orientation_declination);

	// return value definition
	return_outputs.orientation_earth_frame.quaterion = orientation_declination;

	return_outputs.orientation.quaterion = orientation_global_declination;

	return_outputs.orientation_earth_frame.euler = quaternion_to_euler(orientation_declination);
	
	return_outputs.orientation.euler = quaternion_to_euler(orientation_global_declination);

	return_outputs.acceleration.zero = acceleration_zero;

	return_outputs.acceleration.global = acceleration_global;

	return return_outputs;
}


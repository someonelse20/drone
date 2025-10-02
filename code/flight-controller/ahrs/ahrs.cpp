#include <cmath>
#include <vector>
#include <chrono>
#include "ahrs.h"

using namespace std;

settings_struct settings;

vector<double> orientation = {1, 0, 0, 0}; // Orientation as a quaterion. Starts at no rotation.
double gain;
vector<double> error(3), a_error(3), m_error(3);

double start_timestamp = 0;

double accel_t;
double accel_t_timestamp = 0;

double norm(vector<double> v) { // calculates the norm of an array
	double return_value;
	for (int i = 0; i < v.size(); i++) {
		return_value += pow(v[i], 2);
	}
	return sqrt(return_value);
}

vector<double> quaternion_to_euler(vector<double> q) {
	return vector<double> {atan2(2 * (q[2] * q[3] - q[0] * q[1]), 2 * pow(q[0], 2) - 1 + 2 * pow(q[3], 2)),
	                       -asin(2 * (q[1] * q[3] + q[0] * q[2])),
	                       atan2(2 * (q[1] * q[2] - q[0] * q[3]), 2 * pow(q[0], 2) - 1 + 2 * pow(q[1], 2))};
}

vector<double> quaterion_conjugate(vector<double> q) {
	return vector<double> {q[0], -q[1], -q[2], -q[3]};
}

vector<double> quaternion_product(vector<double> qa, vector<double> qb) {
	return vector<double> {qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3],
	                       qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2],
	                       qa[0] * qb[2] - qa[1] * qb[3] + qa[2] * qb[0] + qa[3] * qb[1],
	                       qa[0] * qb[3] + qa[1] * qb[2] - qa[2] * qb[1] + qa[3] * qb[0]};
}

/*vector<double> scale_quaternion(vector<double> q, double scalar) { // Scale_vector should do the same thing.
	return vector<double> {q[0] * scalar, q[1] * scalar, q[2] * scalar, q[3] * scalar};
}*/

vector<double> cross_product(vector<double> va, vector<double> vb) {
    return vector<double> {va[1] * vb[2] - va[2] * vb[1], va[2] * vb[0] - va[0] * vb[2], va[0] * vb[1] - va[1] * vb[0]};
}

vector<double> scale_vector(vector<double> v, double scalar) {
	vector<double> return_vector(v.size());
	for (int i = 0; i < v.size(); i++) {
		return_vector[i] = v[i] * scalar;
	}
	return return_vector;
}

vector<double> add_vector(vector<double> va, vector<double> vb) {
	vector<double> return_vector(va.size());
	for (int i = 0; i < va.size(); ++i) {
	    return_vector[i] = va[i] + vb[i];
	}
	return return_vector;
}

vector<double> subtract_vector(vector<double> va, vector<double> vb) {
	vector<double> return_vector(va.size());
	for (int i = 0; i < va.size(); ++i) {
	    return_vector[i] = va[i] - vb[i];
	}
	return return_vector;
}

double get_timestamp() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

double current_time() {
	return get_timestamp() - start_timestamp;
}

vector<double> gyro_bias_compensation(vector<double> gyro) { // Possibly replace with low pass filter.
	return gyro;
}

vector<double> mag_rejection(vector<double> mag) {
	norm(mag);
	if (settings.min_mag_distortion < norm(mag) && norm(mag) < settings.max_mag_distoriton) {
		 return mag;
	}
	else {
		return vector<double> {0, 0, 0};
	}
}

vector<double> accel_rejection(vector<double> accel){ 
	// calculate the amount of time that the accelerometer measurment is unreliable
	if (accel_t_timestamp == 0)
		accel_t_timestamp = get_timestamp();
	if (-settings.accel_rejection < (norm(accel) - 1) && (norm(accel) - 1) < settings.accel_rejection) 
		accel_t += get_timestamp() - accel_t_timestamp;
	else 
		accel_t = 0;

	if (accel_t > settings.accel_rejection_t) {
		return vector<double> {0, 0, 0};
	}
	else {
		return accel;
	}
}

outputs update(vector<double> accel, vector<double> gyro, vector<double> mag, double dt) {
	outputs return_outputs;

	// set timestamp
	if (start_timestamp == 0)
		start_timestamp = get_timestamp();

	// initialise 
	if (current_time() < settings.init_time) 
		gain = settings.gain_init + (pow(settings.gain_init, -current_time()) / settings.gain_init) * (settings.gain_init - settings.gain_normal);
	else
		gain = settings.gain_normal;
	
	// sensor conditioning
	vector<double> gyro_conditioned = gyro_bias_compensation(gyro);
	vector<double> accel_conditioned = accel_rejection(accel);
	vector<double> mag_conditioned = mag_rejection(mag);

	// error calculation
	vector<double> accel_normalized = scale_vector(accel_conditioned, 1/norm(accel_conditioned));
	a_error = cross_product(accel_normalized, vector<double> {(2 * orientation[1] * orientation[3]) - (2 * orientation[0] * orientation[2]),
                                                              (2 * orientation[2] * orientation[3]) + (2 * orientation[0] * orientation[1]),
                                                              (2 * pow(orientation[0], 2)) - 1 + (2 * pow(orientation[3], 2))});
	vector<double> cross_a_m = cross_product(accel_normalized, scale_vector(mag_conditioned, 1/norm(mag_conditioned)));
	m_error = cross_product(cross_a_m, vector<double> {(-2 * pow(orientation[0], 2)) + 1 - (2 * pow(orientation[0], 2)),
	                                                   (-2 * orientation[1] * orientation[2]) + (2 * orientation[0] * orientation[3]),
	                                                   (-2 * orientation[1] * orientation[3]) - (2 * orientation[0] * orientation[2])});
	
	if (norm(accel_conditioned) > 0 && norm(mag_conditioned) > 0)
		error = add_vector(a_error, m_error);
	else if (norm(accel_conditioned) > 0)
		error = a_error;
	else 
		error = vector<double> {0, 0, 0};

	// complementary filter
	vector<double> gyro_error_product = subtract_vector(gyro_conditioned, scale_vector(error, gain));
	vector<double> orientation_rate_of_chage = scale_vector(quaternion_product(orientation, vector<double> {0, gyro_error_product[0], gyro_error_product[1], gyro_error_product[2]}), 0.5);

	vector<double> orientation_unnormalised = add_vector(orientation, scale_vector(orientation_rate_of_chage, dt));
	orientation = scale_vector(orientation_unnormalised, 1 / norm(orientation_unnormalised));

	// zero g acceleration calculation (Don't use the rejected acceleration results here.)
	vector<double> acceleration_zero = subtract_vector(accel, vector<double> {2 * orientation[1] * orientation[3] - 2 * orientation[0] * orientation[2],
	                                                                          2 * orientation[2] * orientation[3] + 2 * orientation[0] * orientation[1],
	                                                                          2 * pow(orientation[0], 2) - 1 + 2 * pow(orientation[3], 2)});
	vector<double> orientation_global = quaterion_conjugate(orientation);
	vector<double> acceleration_global_quaternion = quaternion_product(quaternion_product(orientation, vector<double> {0, acceleration_zero[0], acceleration_zero[1], acceleration_zero[2]}),
	                                                                   orientation_global);
	vector<double> acceleration_global = vector<double> {acceleration_global_quaternion[1], acceleration_global_quaternion[2], acceleration_global_quaternion[3]};

	// return value definition
	return_outputs.orientation_earth_frame.quaterion.w = orientation[0];
	return_outputs.orientation_earth_frame.quaterion.x = orientation[1];
	return_outputs.orientation_earth_frame.quaterion.y = orientation[2];
	return_outputs.orientation_earth_frame.quaterion.z = orientation[3];

	return_outputs.orientation.quaterion.w = orientation_global[0];
	return_outputs.orientation.quaterion.x = orientation_global[1];
	return_outputs.orientation.quaterion.y = orientation_global[2];
	return_outputs.orientation.quaterion.z = orientation_global[3];

	vector<double> orientation_euler = quaternion_to_euler(orientation);

	return_outputs.orientation_earth_frame.euler.x = orientation_euler[0];
	return_outputs.orientation_earth_frame.euler.y = orientation_euler[1];
	return_outputs.orientation_earth_frame.euler.z = orientation_euler[2];
	
	vector<double> orientation_global_euler = quaternion_to_euler(orientation_global);

	return_outputs.orientation.euler.x = orientation_global_euler[0];
	return_outputs.orientation.euler.y = orientation_global_euler[1];
	return_outputs.orientation.euler.z = orientation_global_euler[2];

	return_outputs.acceleration.zero.x = acceleration_zero[0];
	return_outputs.acceleration.zero.y = acceleration_zero[1];
	return_outputs.acceleration.zero.z = acceleration_zero[2];

	return_outputs.acceleration.global.x = acceleration_global[0];
	return_outputs.acceleration.global.y = acceleration_global[1];
	return_outputs.acceleration.global.z = acceleration_global[2];

	return return_outputs;
}

void set_settings(settings_struct new_settings) {
	settings.gain_normal = new_settings.gain_normal;
	settings.gain_init = new_settings.gain_init;
	settings.init_time = new_settings.init_time;
	settings.min_mag_distortion = new_settings.min_mag_distortion;
	settings.max_mag_distoriton = new_settings.max_mag_distoriton;
	settings.accel_rejection = new_settings.accel_rejection;
	settings.accel_rejection_t = new_settings.accel_rejection_t;
	settings.gyro_calibrate = new_settings.gyro_calibrate;
	settings.accel_calibrate = new_settings.accel_calibrate;
	settings.mag_calibrate = new_settings.mag_calibrate;
}


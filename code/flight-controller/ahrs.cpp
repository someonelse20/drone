#include <cmath>
#include <chrono>
#include "ahrs.h"
#include <iostream>

using namespace std;

double ahrs::vector_norm(vector_struct e) {
	return sqrt(pow(e.x, 2) + pow(e.y, 2) + pow(e.z, 2));
}

double ahrs::quaternion_norm(quaternion_struct q) {
	return sqrt(pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2) + pow(q.w, 2));
}

vector_struct ahrs::quaternion_to_euler(quaternion_struct q) {
	vector_struct return_value;

	return_value.x = atan2(2 * (q.y * q.z - q.w * q.x), 2 * pow(q.w, 2) - 1 + 2 * pow(q.z, 2)) * 180.0 / M_PI,
	return_value.y = -asin(2 * (q.x * q.z + q.w * q.y)) * 180.0 / M_PI,
	return_value.z = atan2(2 * (q.x * q.y - q.w * q.z), 2 * pow(q.w, 2) - 1 + 2 * pow(q.x, 2)) * 180.0 / M_PI;

	return return_value;
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

double ahrs::get_timestamp() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() / 1000.0;
}

double ahrs::current_time() {
	return get_timestamp() - start_timestamp;
}

vector_struct ahrs::gyro_bias_compensation(vector_struct gyro) { // Possibly replace with low pass filter.
	return gyro;
}

vector_struct ahrs::mag_rejection(vector_struct mag) {
	vector_norm(mag);
	if (-settings.min_mag_distortion < vector_norm(mag) && vector_norm(mag) < settings.max_mag_distoriton) {
		vector_struct return_value;

		return_value.x = 0;
		return_value.y = 0;
		return_value.z = 0;

		return return_value;
	}
	else {
		return mag;
	}
}

vector_struct ahrs::accel_rejection(vector_struct accel){ 
	// calculate the amount of time that the accelerometer measurment is unreliable
	if (accel_t_timestamp == 0)
		accel_t_timestamp = get_timestamp();
	if ((vector_norm(accel) - 1) > -settings.accel_rejection && (vector_norm(accel) - 1) < settings.accel_rejection) 
		accel_t = 0;
	else 
		accel_t += get_timestamp() - accel_t_timestamp;

	if (accel_t > settings.accel_rejection_t) {
		vector_struct return_value;

		return_value.x = 0;
		return_value.y = 0;
		return_value.z = 0;

		return return_value;
	}
	else {
		return accel;
	}
}

output_struct ahrs::update(vector_struct gyro, vector_struct accel, vector_struct mag, double dt) {
	output_struct return_outputs;

	// set timestamp
	if (start_timestamp == -1) {
		start_timestamp = get_timestamp();
	}


	// initialise 
	if (current_time() < settings.init_time) 
		gain = settings.gain_normal + (pow(settings.init_time, -current_time()) / settings.init_time) * (settings.gain_init - settings.gain_normal);
	else
		gain = settings.gain_normal;

	// sensor conditioning
	vector_struct gyro_conditioned = gyro_bias_compensation(gyro);
	vector_struct accel_conditioned = accel_rejection(accel);
	vector_struct mag_conditioned = mag_rejection(mag);

	cout << gyro_conditioned.x << "," << gyro_conditioned.y << "," << gyro_conditioned.z << endl;

	// error calculation
	vector_struct accel_normalized = scale_vector(accel_conditioned, 1/vector_norm(accel_conditioned));
	a_error = cross_product(accel_normalized, vector_struct {(2 * orientation.x * orientation.z) - (2 * orientation.w * orientation.y),
                                                             (2 * orientation.y * orientation.z) + (2 * orientation.w * orientation.x),
                                                             (2 * pow(orientation.w, 2)) - 1 + (2 * pow(orientation.z, 2))});
	vector_struct cross_a_m = cross_product(accel_normalized, scale_vector(mag_conditioned, 1/vector_norm(mag_conditioned)));
	m_error = cross_product(cross_a_m, vector_struct {(-2 * pow(orientation.w, 2)) + 1 - (2 * pow(orientation.x, 2)),
                                                      (-2 * orientation.x * orientation.y) + (2 * orientation.w * orientation.z),
                                                      (-2 * orientation.x * orientation.z) - (2 * orientation.w * orientation.y)});
	
	if (vector_norm(accel_conditioned) > 0 && vector_norm(mag_conditioned) > 0)
		error = add_vector(a_error, m_error);
	else if (vector_norm(accel_conditioned) > 0)
		error = a_error;
	else 
		error = vector_struct {0, 0, 0};

	// cout << a_error.x << "," << a_error.y << "," << a_error.z << endl;
	// cout << error.x << "," << error.y << "," << error.z << endl;

	// complementary filter
	vector_struct gyro_error_product = subtract_vector(gyro_conditioned, scale_vector(error, gain));

	// cout << gyro_error_product.x << "," << gyro_error_product.y << "," << gyro_error_product.z << endl;

	quaternion_struct orientation_rate_of_chage = scale_quaternion(quaternion_product(orientation, quaternion_struct {0, gyro_error_product.x, gyro_error_product.y, gyro_error_product.z}), 0.5);

	quaternion_struct orientation_unnormalised = add_quaternion(orientation, scale_quaternion(orientation_rate_of_chage, dt));
	orientation = scale_quaternion(orientation_unnormalised, 1 / quaternion_norm(orientation_unnormalised));

	// zero g acceleration calculation (Don't use the rejected acceleration results here.)
	vector_struct acceleration_zero = subtract_vector(accel, vector_struct {2 * orientation.x * orientation.z - 2 * orientation.w * orientation.y,
	                                                                          2 * orientation.y * orientation.z + 2 * orientation.w * orientation.x,
	                                                                          2 * pow(orientation.w, 2) - 1 + 2 * pow(orientation.z, 2)});
	quaternion_struct orientation_global = quaternion_conjugate(orientation);
	quaternion_struct acceleration_global_quaternion = quaternion_product(quaternion_product(orientation, quaternion_struct {0, acceleration_zero.x, acceleration_zero.y, acceleration_zero.z}),
	                                                                   orientation_global);
	vector_struct acceleration_global = vector_struct {acceleration_global_quaternion.x, acceleration_global_quaternion.y, acceleration_global_quaternion.z};

	// return value definition
	return_outputs.orientation_earth_frame.quaterion = orientation;

	return_outputs.orientation.quaterion = orientation_global;

	return_outputs.orientation_earth_frame.euler = quaternion_to_euler(orientation);
	
	return_outputs.orientation.euler = quaternion_to_euler(orientation_global);

	return_outputs.acceleration.zero = acceleration_zero;

	return_outputs.acceleration.global = acceleration_global;

	return return_outputs;
}


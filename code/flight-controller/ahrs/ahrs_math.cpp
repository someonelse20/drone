#include <string>
#include <cmath>

#include "ahrs.h"

quaternion_struct quaternion_struct::operator+(const quaternion_struct &v) {
	quaternion_struct return_value;

	return_value.w = w + v.w;
	return_value.x = x + v.x;
	return_value.y = y + v.y;
	return_value.z = z + v.z;

	return return_value;
}

quaternion_struct quaternion_struct::operator-(const quaternion_struct &v) {
	quaternion_struct return_value;

	return_value.w = w - v.w;
	return_value.x = x - v.x;
	return_value.y = y - v.y;
	return_value.z = z - v.z;

	return return_value;
}

quaternion_struct quaternion_struct::operator*(const quaternion_struct &q) {
	quaternion_struct return_value;

	return_value.w = w * q.w - x * q.x - y * q.y - z * q.z;
	return_value.x = w * q.x + x * q.w + y * q.z - z * q.y;
	return_value.y = w * q.y - x * q.z + y * q.w + z * q.x;
	return_value.z = w * q.z + x * q.y - y * q.x + z * q.w;

	return return_value;
}

quaternion_struct quaternion_struct::operator*(const double &s) {
	quaternion_struct return_value;

	return_value.w = w * s;
	return_value.x = x * s;
	return_value.y = y * s;
	return_value.z = z * s;

	return return_value;
}

bool quaternion_struct::operator==(const quaternion_struct &q) {
	return w == q.w && x == q.x && y == q.y && z == q.z;
}

bool quaternion_struct::operator!=(const quaternion_struct &q) {
	return w != q.w || x != q.x || y != q.y || z != q.z;
}

quaternion_struct quaternion_struct::conjugate() {
	quaternion_struct return_value;

	return_value.w = w;
	return_value.x = -x;
	return_value.y = -y;
	return_value.z = -z;

	return return_value;
}

double quaternion_struct::norm() {
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(w, 2));
}

quaternion_struct quaternion_struct::normalize() {
	return operator*(1 / norm());
}

std::string quaternion_struct::to_string(const std::string seperator) {
	return std::to_string(w) + seperator + std::to_string(x) + seperator + std::to_string(y) + seperator + std::to_string(z);
}

vector_struct quaternion_struct::to_euler() {
	vector_struct return_value;

	return_value.x = atan2(2 * (y * z - w * x), 2 * pow(w, 2) - 1 + 2 * pow(z, 2)) * (180.0 / M_PI),
	return_value.y = -asin(2 * (x * z + w * y)) * 180.0 / M_PI,
	return_value.z = atan2(2 * (x * y - w * z), 2 * pow(w, 2) - 1 + 2 * pow(x, 2)) * (180.0 / M_PI);

	return return_value;
}

vector_struct vector_struct::operator+(const vector_struct &v) {
	vector_struct return_value;

	return_value.x = x + v.x;
	return_value.y = y + v.y;
	return_value.z = z + v.z;

	return return_value;
}

vector_struct vector_struct::operator-(const vector_struct &v) {
	vector_struct return_value;

	return_value.x = x - v.x;
	return_value.y = y - v.y;
	return_value.z = z - v.z;

	return return_value;
}

vector_struct vector_struct::operator*(const vector_struct &v) {
	vector_struct return_value;

	return_value.x = y * v.z - z * v.y;
	return_value.y = z * v.x - x * v.z;
	return_value.z = x * v.y - y * v.x;

	return return_value;
}

vector_struct vector_struct::operator*(const double &s) {
	vector_struct return_value;

	return_value.x = x * s;
	return_value.y = y * s;
	return_value.z = z * s;

	return return_value;
}

bool vector_struct::operator==(const vector_struct &v) {
	return x == v.x && y == v.y && z == v.z;
}

bool vector_struct::operator!=(const vector_struct &v) {
	return x != v.x || y != v.y || z != v.z;
}

double vector_struct::norm() {
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

vector_struct vector_struct::normalize() {
	return operator*(1 / norm());
}

quaternion_struct vector_struct::to_quaternion() {
	quaternion_struct return_value;

	// abbreviations for the various angular functions
	double cr = cos(x * 0.5);
	double sr = sin(x * 0.5);
	double cp = cos(y * 0.5);
	double sp = sin(y * 0.5);
	double cy = cos(z * 0.5);
	double sy = sin(z * 0.5);

	return_value.w = cr * cp * cy + sr * sp * sy;
	return_value.x = sr * cp * cy - cr * sp * sy;
	return_value.y = cr * sp * cy + sr * cp * sy;
	return_value.z = cr * cp * sy - sr * sp * cy;

	return return_value;
}

std::string vector_struct::to_string(const std::string seperator) {
	return std::to_string(x) + seperator + std::to_string(y) + seperator + std::to_string(z);
}

matrix_struct matrix_struct::operator*(const matrix_struct &m) {
	matrix_struct return_value;

	return_value.x.x = x.x * m.x.x + x.y * m.y.x + x.z * m.z.x;
	return_value.x.y = x.x * m.x.y + x.y * m.y.y + x.z * m.z.y;
	return_value.x.z = x.x * m.x.z + x.y * m.y.z + x.z * m.z.z;

	return_value.y.x = y.x * m.x.x + y.y * m.y.x + y.z * m.z.x;
	return_value.y.y = y.x * m.x.y + y.y * m.y.y + y.z * m.z.y;
	return_value.y.z = y.x * m.x.z + y.y * m.y.z + y.z * m.z.z;

	return_value.z.x = z.x * m.x.x + z.y * m.y.x + z.z * m.z.x;
	return_value.z.y = z.x * m.x.y + z.y * m.y.y + z.z * m.z.y;
	return_value.z.z = z.x * m.x.z + z.y * m.y.z + z.z * m.z.z;

	return return_value;
}

vector_struct matrix_struct::operator*(const vector_struct &v) {
	vector_struct return_value;

	return_value.x = x.x * v.x + x.y * v.y + x.z * v.z;
	return_value.y = y.x * v.x + y.y * v.y + y.z * v.z;
	return_value.z = z.x * v.x + z.y * v.y + z.z * v.z;

	return return_value;
}

bool matrix_struct::operator==(const matrix_struct &m) {
	return x == m.x && y == m.y && z == m.z;
}

bool matrix_struct::operator!=(const matrix_struct &m) {
	return x != m.x || y != m.y || z != m.z;
}

std::string matrix_struct::to_string(const std::string seperator, const std::string vector_seperator) {
	return x.to_string(vector_seperator) + seperator + y.to_string(vector_seperator) + seperator + z.to_string(vector_seperator);
}


#pragma once
#include <stdint.h>

#define _G_ 9.8

struct total_value_t {
	double min;
	double max;
	double avg;
	double std;
	double sum;
	double square_diff_sum;
};
struct imu_total_t {
	total_value_t accel[3]; // x, y, z
	total_value_t gyro[3]; // x, y, z
	uint32_t last_time;
};
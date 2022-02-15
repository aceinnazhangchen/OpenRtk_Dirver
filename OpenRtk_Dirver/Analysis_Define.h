#pragma once
#include <stdint.h>

#define _G_ 9.8

#pragma pack(push, 1)
struct static_gnss_t {
	uint16_t	gps_week;
	uint32_t	gps_millisecs;
	uint8_t		position_type;
	double		latitude;
	double		longitude;
	double		height;
	float		north_vel;
	float		east_vel;
	float		up_vel;
};

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

#pragma pack(pop)
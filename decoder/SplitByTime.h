#pragma once
#include <stdint.h>
#include <vector>

struct ins_sol_data {
	uint16_t	gps_week;
	uint32_t	gps_millisecs;
	uint8_t		ins_status;
	uint8_t		ins_position_type;
	double		latitude;
	double		longitude;
	double		height;
	float		north_velocity;
	float		east_velocity;
	float		up_velocity;
	float		roll;
	float		pitch;
	float		heading;
};

struct stTimeSlice {
	uint16_t	week;
	uint32_t	starttime;
	uint32_t	endtime;
	uint32_t	currenttime;
	double		currentspeed;
	uint32_t	during;
	double		startpos[3];
	double		endpos[3];
	double		dst_s;
	double		dst_e;
	double		distance;
	double		speed_distance;
	uint8_t		used_in_roll;
	double		start_heading;
	double		end_heading;
	double		last_heading;
	double		angle_diff;
};

class SplitByTime
{
public:
	SplitByTime();
	~SplitByTime();
	void init();
	void set_min_distance(int distance);
	std::vector<stTimeSlice>& get_time_slices();
	void calc_distance_between_start_end();
	int input_sol_data(ins_sol_data& ins_data);
	void filter_section();
	void finish();
	stTimeSlice* get_current_slice();
private:
	stTimeSlice m_current_time;
	ins_sol_data m_last_ins_data;
	std::vector<stTimeSlice> time_slices;
	int m_min_distance_limit;
};

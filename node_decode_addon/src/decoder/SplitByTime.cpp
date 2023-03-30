#include "SplitByTime.h"
#include "common.h"
#include "model.h"
#include <math.h>

#define MIN_DRING_TIME 5

SplitByTime::SplitByTime()
{
	m_min_distance_limit = 200;
	init();
}

SplitByTime::~SplitByTime()
{
}

void SplitByTime::init()
{
	memset(&m_last_ins_data, 0, sizeof(ins_sol_data));
	memset(&m_current_time, 0, sizeof(stTimeSlice));
	time_slices.clear();
}

std::vector<stTimeSlice>& SplitByTime::get_time_slices()
{
	return time_slices;
}

void SplitByTime::set_min_distance(int distance)
{
	m_min_distance_limit = distance;
}

void SplitByTime::calc_distance_between_start_end() {
	double startxyz[3] = { 0.0 };
	double endxyz[3] = { 0.0 };
	blh2xyz(m_current_time.startpos, startxyz);
	blh2xyz(m_current_time.endpos, endxyz);
	m_current_time.distance = distance(startxyz, endxyz);
}

int SplitByTime::input_sol_data(ins_sol_data & ins_data)
{
	int start_or_end = 0; // start:1 , end:2
	if (ins_data.gps_millisecs % 1000 >= 100) {
		start_or_end = -1;
		return start_or_end;
	}
	if (ins_data.gps_millisecs - m_last_ins_data.gps_millisecs < 1000) {
		start_or_end = -1;
		return start_or_end;
	}
	float angle = ins_data.heading - m_last_ins_data.heading;
	while (angle > 180) {
		angle = angle - 360;
	}
	while (angle < -180) {
		angle = angle + 360;
	}
	if (m_current_time.starttime == 0) {
		memset(&m_current_time, 0, sizeof(stTimeSlice));
		if (fabs(angle) < 5) {
			if (ins_data.ins_position_type == 4) {
				float speed = sqrt(ins_data.east_velocity * ins_data.east_velocity + ins_data.north_velocity * ins_data.north_velocity);
				if (speed >= 5) {
					//start
					m_current_time.week = ins_data.gps_week;
					m_current_time.currentspeed = speed;
					m_current_time.starttime = ins_data.gps_millisecs;
					m_current_time.currenttime = ins_data.gps_millisecs;
					m_current_time.startpos[0] = ins_data.latitude * D2R;
					m_current_time.startpos[1] = ins_data.longitude * D2R;
					m_current_time.startpos[2] = ins_data.height;
					m_current_time.start_heading = ins_data.heading;
					if (time_slices.size() > 0) {
						m_current_time.last_heading = time_slices[time_slices.size() - 1].end_heading;
						m_current_time.angle_diff = m_current_time.start_heading - m_current_time.last_heading;
						while (m_current_time.angle_diff > 180) {
							m_current_time.angle_diff = m_current_time.angle_diff - 360;
						}
						while (m_current_time.angle_diff < -180) {
							m_current_time.angle_diff = m_current_time.angle_diff + 360;
						}
						// if (fabs(m_current_time.angle_diff) <= 75) {// ????
						// 	memset(&m_current_time, 0, sizeof(stTimeSlice));
						// }
					}
					// printf("time= %d, %d, angle = %9.3f, %9.3f, %9.3f, speed = %f\n", m_last_ins_data.gps_millisecs/1000, ins_data.gps_millisecs/1000, angle, m_last_ins_data.heading, ins_data.heading,speed);
					start_or_end = 1;
				}
			}
		}
	}
	else if (m_current_time.starttime != 0 && m_current_time.endtime == 0)
	{
		m_current_time.currenttime = ins_data.gps_millisecs;
		float speed = sqrt(ins_data.east_velocity * ins_data.east_velocity + ins_data.north_velocity * ins_data.north_velocity);
		double distance = sqrt(0.25*(m_last_ins_data.north_velocity + m_last_ins_data.north_velocity)*(ins_data.north_velocity + ins_data.north_velocity)
			+ 0.25*(m_last_ins_data.east_velocity + m_last_ins_data.east_velocity)*(ins_data.east_velocity + ins_data.east_velocity));
		m_current_time.currentspeed = speed;
		m_current_time.speed_distance += distance;
		if (ins_data.ins_position_type != 4 || speed < 5 || fabs(angle) >= 5) {
			//end
			m_current_time.endtime = m_last_ins_data.gps_millisecs;
		}
	}
	if (m_current_time.starttime != 0 && m_current_time.endtime != 0){
		if (m_current_time.endtime > m_current_time.starttime) {
			m_current_time.endpos[0] = m_last_ins_data.latitude * D2R;
			m_current_time.endpos[1] = m_last_ins_data.longitude * D2R;
			m_current_time.endpos[2] = m_last_ins_data.height;
			m_current_time.during = m_current_time.endtime - m_current_time.starttime;
			m_current_time.end_heading = m_last_ins_data.heading;
			if (m_current_time.during >= MIN_DRING_TIME * 1000) {
				// calc_distance_between_start_end();
				if (m_current_time.speed_distance >= m_min_distance_limit) {
					time_slices.push_back(m_current_time);
					start_or_end = 2;
				}
			}
		}		
		m_current_time.starttime = 0;
	}
	m_last_ins_data = ins_data;
	return start_or_end;
}

void SplitByTime::filter_section() {
	std::vector<int> section_ids;
	int used_num = 0;
	for (int i = 1; i < time_slices.size(); i++) {
		double startxyz[3] = { 0.0 };
		double endxyz[3] = { 0.0 };
		blh2xyz(time_slices[i].startpos, startxyz);
		blh2xyz(time_slices[i].endpos, endxyz);

		double last_startxyz[3] = { 0.0 };
		double last_endxyz[3] = { 0.0 };
		blh2xyz(time_slices[i - 1].startpos, last_startxyz);
		blh2xyz(time_slices[i - 1].endpos, last_endxyz);

		time_slices[i].dst_s = distance(startxyz, last_endxyz);
		time_slices[i].dst_e = distance(endxyz, last_startxyz);
		if (time_slices[i].dst_s < 50.0 && time_slices[i].dst_e < 50.0) {
			time_slices[i-1].used_in_roll = 1;
			time_slices[i].used_in_roll = 1;
		}
		else {
			time_slices[i].used_in_roll = 0;
		}
	}
}

void SplitByTime::finish()
{
	 if (m_current_time.starttime != 0 && m_current_time.endtime == 0)
	 {
	 	if (m_last_ins_data.ins_position_type == 4 && m_last_ins_data.gps_millisecs > 0) {
	 		m_current_time.endtime = m_last_ins_data.gps_millisecs;
	 		m_current_time.during = m_current_time.endtime - m_current_time.starttime;
	 		if (m_current_time.during >= MIN_DRING_TIME * 1000) {
				if (m_current_time.speed_distance >= m_min_distance_limit) {
					time_slices.push_back(m_current_time);
				}
	 		}
	 	}
	 }
	memset(&m_current_time, 0, sizeof(stTimeSlice));
	filter_section();
}

stTimeSlice * SplitByTime::get_current_slice()
{
	return &m_current_time;
}

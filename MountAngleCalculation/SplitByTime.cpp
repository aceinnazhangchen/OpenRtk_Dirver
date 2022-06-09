#include "SplitByTime.h"

SplitByTime::SplitByTime(QObject *parent)
	: QObject(parent)
{
	init();
}

SplitByTime::~SplitByTime()
{
}

void SplitByTime::init()
{
	m_last_position_type = 0;
	m_last_time = 0;
	memset(&m_current_time, 0, sizeof(stTimeSlice));
	time_slices.clear();
}

std::vector<stTimeSlice>& SplitByTime::get_time_slices()
{
	// TODO: 在此处插入 return 语句
	return time_slices;
}

void SplitByTime::input_sol_data(ins_sol_data & ins_data)
{
	if (m_current_time.starttime == 0) {
		if (ins_data.ins_position_type == 4) {
			m_current_time.week = ins_data.gps_week;
			m_current_time.starttime = ins_data.gps_millisecs;
		}
	}
	else if (m_current_time.starttime != 0 && m_current_time.endtime == 0)
	{
		if (ins_data.ins_position_type != 4) {
			m_current_time.endtime = ins_data.gps_millisecs;
			m_current_time.during = m_current_time.endtime - m_current_time.starttime;
		}
	}
	if (m_current_time.starttime != 0 && m_current_time.endtime != 0)
	{
		time_slices.push_back(m_current_time);
		memset(&m_current_time, 0, sizeof(stTimeSlice));
	}
	m_last_time = ins_data.gps_millisecs;
	m_last_position_type = ins_data.ins_position_type;
}

void SplitByTime::finish()
{
	if (m_current_time.starttime != 0 && m_current_time.endtime == 0)
	{
		if (m_last_position_type == 4 && m_last_time > 0) {
			m_current_time.endtime = m_last_time;
			m_current_time.during = m_current_time.endtime - m_current_time.starttime;
			time_slices.push_back(m_current_time);
			memset(&m_current_time, 0, sizeof(stTimeSlice));
		}
	}
}
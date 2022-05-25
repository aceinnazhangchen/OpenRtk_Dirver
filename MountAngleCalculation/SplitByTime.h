#pragma once

#include <QObject>
#include "DriveStatus.h"

struct stTimeSlice {
	uint16_t	week;
	uint32_t	starttime;	// second
	uint32_t	endtime;	// second
	uint32_t	during;		// second
};

class SplitByTime : public QObject
{
	Q_OBJECT

public:
	SplitByTime(QObject *parent);
	~SplitByTime();
	void init();
	std::vector<stTimeSlice>& get_time_slices();
	void input_sol_data(ins_sol_data& ins_data);
	void finish();
private:
	uint8_t m_last_position_type;
	uint32_t m_last_time;
	stTimeSlice m_current_time;
	std::vector<stTimeSlice> time_slices;
};

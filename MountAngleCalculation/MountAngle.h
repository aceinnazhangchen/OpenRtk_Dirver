#pragma once

#include <QObject>
#include "DriveStatus.h"
#include <vector>

struct stTimeSlice {
	uint32_t	starttime;	// second
	uint32_t	endtime;	// second
	uint32_t	during;		// second
};

class MountAngle : public QObject
{
	Q_OBJECT

public:
	MountAngle(QObject *parent);
	~MountAngle();
	void init();
	void set_base_file_name(char* file_name);
	void process_ins();
	void mountangle_process();
	void process_live_data(ins_sol_data& ins_data);
	void create_file(FILE *& file, const char * suffix, const char * title);
	void finish();
	std::vector<stTimeSlice>& get_time_slices();
private:
	DriveStatus* drive_status;
	liveresult_t last_drive_res;
	char base_file_name[256];
	FILE* f_log;
	FILE* f_time_log;
	std::vector<stTimeSlice> time_slices;
};
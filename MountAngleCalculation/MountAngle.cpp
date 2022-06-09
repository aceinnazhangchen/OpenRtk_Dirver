#include "MountAngle.h"
#include "CalculationCall.h"

MountAngle::MountAngle(QObject *parent)
	: QObject(parent)
{
	drive_status = new DriveStatus();
	init();
}

MountAngle::~MountAngle()
{
	delete drive_status;
}
void MountAngle::init()
{
	memset(base_file_name, 0, 256);
	f_log = NULL;
	f_time_log = NULL;
	drive_status->init();
	time_slices.clear();
}
void MountAngle::set_base_file_name(char * file_name)
{
	strcpy(base_file_name, file_name);
}

void MountAngle::process_ins() {
	CalculationCall::call_ins_start("./content_aceinna_config.json");
}

void MountAngle::mountangle_process() {
	liveresult_t* drive_res = drive_status->getresult();
	if (!drive_res)return;
	if (drive_res->type == 14) {
		create_file(f_time_log, "mountangle.time", NULL);
		uint32_t starttime = drive_res->starttime + 3000;
		uint32_t endtime = drive_res->curtime;
		stTimeSlice time_slice = { 0 };
		time_slice.week = drive_res->gps_week;
		time_slice.starttime = starttime / 1000;
		time_slice.endtime = endtime / 1000;
		time_slice.during = time_slice.endtime - time_slice.starttime;
		if (f_time_log) fprintf(f_time_log, "%d,%d,%d,%d\n", time_slice.week,time_slice.starttime, time_slice.endtime, time_slice.during);
		time_slices.push_back(time_slice);
	}
	else if (drive_res->type == 15) {
		create_file(f_time_log, "mountangle.time", NULL);
		uint32_t starttime = drive_res->starttime + 5000;
		uint32_t endtime = drive_res->curtime - 5000;
		stTimeSlice time_slice = { 0 };
		time_slice.week = drive_res->gps_week;
		time_slice.starttime = starttime / 1000;
		time_slice.endtime = endtime / 1000;
		time_slice.during = time_slice.endtime - time_slice.starttime;
		if (f_time_log) fprintf(f_time_log, "%d,%d,%d,%d\n", time_slice.week, time_slice.starttime, time_slice.endtime, time_slice.during);
		time_slices.push_back(time_slice);
		starttime = drive_res->starttime + 3000;
		endtime = drive_res->curtime;
		time_slice.starttime = starttime / 1000;
		time_slice.endtime = endtime / 1000;
		time_slice.during = time_slice.endtime - time_slice.starttime;
		if (f_time_log) fprintf(f_time_log, "%d,%d,%d,%d\n", time_slice.week,time_slice.starttime, time_slice.endtime, time_slice.during);
		time_slices.push_back(time_slice);
	}
}

void MountAngle::process_live_data(ins_sol_data & ins_data)
{
	drive_status->addrawdata(&ins_data);
	liveresult_t* drive_res =  drive_status->getresult();
	if (drive_res) {
		create_file(f_log, "mountangle.log", NULL);
		if (f_log) fprintf(f_log, "%.1f:{%d,%f,%f,%f,%d,%d}\n", (double)ins_data.gps_millisecs/1000.0,drive_res->type, drive_res->totalangle_r, drive_res->totalangle_l, drive_res->distance, drive_res->starttime, drive_res->curtime);
		if ((last_drive_res.type == 13 && drive_res->type == 14)
			|| (last_drive_res.type == 14 && drive_res->type == 15)){
			mountangle_process();
		}
		memcpy(&last_drive_res, drive_res, sizeof(liveresult_t));
	}
}

void MountAngle::create_file(FILE *& file, const char * suffix, const char * title)
{
	if (strlen(base_file_name) == 0) return;
	if (file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s", base_file_name, suffix);
		file = fopen(file_name, "wb");
		if (file && title) {
			fprintf(file, title);
		}
	}
}

void MountAngle::finish()
{
	//process_ins();
	if (f_log) fclose(f_log); f_log = NULL;
	if (f_time_log) fclose(f_time_log); f_time_log = NULL;
}

std::vector<stTimeSlice>& MountAngle::get_time_slices()
{
	return time_slices;
}

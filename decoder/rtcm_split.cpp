#include "rtcm_split.h"
#include <string.h>

#define HOUR 3600

Rtcm_Split::Rtcm_Split()
{
	init();
}

Rtcm_Split::~Rtcm_Split()
{
}

void Rtcm_Split::init()
{
	split_start_time = 0;
	last_time = 0;
	memset(base_file_name, 0, 256);
	memset(&rtcm, 0, sizeof(rtcm));
	memset(&obs, 0, sizeof(obs));
	memset(&nav, 0, sizeof(nav));
	memset(rtcm_buffer, 0, RTCM_BUFF_SIZE);
	rtcm_buffer_cur = 0;
	files_map.clear();
	nav_file = NULL;
	log_file = NULL;
	time_split_file = NULL;
}

void Rtcm_Split::set_base_file_name(char * file_name)
{
	strcpy(base_file_name, file_name);
}

void Rtcm_Split::close_files()
{
	std::map<uint32_t, FILE*>::iterator it;
	for (it = files_map.begin(); it != files_map.end(); it++) {
		fclose((FILE*)it->second);
	}
	if (nav_file)fclose(nav_file); nav_file = NULL;
	if (log_file)fclose(log_file); log_file = NULL;
	if (time_split_file)fclose(time_split_file); time_split_file = NULL;
}

void Rtcm_Split::input_data(uint8_t data)
{
	//set_approximate_time(2021, 312, &rtcm);
	int ret = 0;
	ret = input_rtcm3_data(&rtcm,data, &obs, &nav);
	if (is_complete_rtcm()) {
		int size = rtcm.len + 3;
		if (rtcm_buffer_cur + size < RTCM_BUFF_SIZE) {
			memcpy(&rtcm_buffer[rtcm_buffer_cur], rtcm.buff, size);
			rtcm_buffer_cur += size;
		}
	}
	if (ret) {
		//if (ret == 2) {
		//	if (nav_file == NULL) {
		//		char file_path[256] = { 0 };
		//		sprintf(file_path, "%s_nav.rtcm", base_file_name);
		//		nav_file = fopen(file_path, "wb");
		//	}
		//	fwrite(rtcm_buffer, 1, rtcm_buffer_cur, nav_file);
		//}
		//else if (obs.staid) {
		//	std::map<uint32_t, FILE*>::iterator it;
		//	it = files_map.find(obs.staid);
		//	if (it == files_map.end()) {
		//		char file_path[256] = { 0 };
		//		sprintf(file_path, "%s_%d.rtcm", base_file_name, obs.staid);
		//		FILE* file = fopen(file_path, "wb");
		//		files_map[obs.staid] = file;
		//	}
		//	FILE* sta_file = files_map[obs.staid];
		//	fwrite(rtcm_buffer, 1, rtcm_buffer_cur, sta_file);
		//}
		if (log_file == NULL) {
			char file_path[256] = { 0 };
			sprintf(file_path, "%s_out.log", base_file_name);
			log_file = fopen(file_path, "wb");
		}
		if (obs.time.time > 0 && obs.time.time >= 1636369200 && obs.time.time <= 1636370100) {
			if (obs.time.time - split_start_time >= HOUR) {
				split_start_time = obs.time.time;
				create_new_split_file();				
			}
			fwrite(rtcm_buffer, 1, rtcm_buffer_cur, time_split_file);
		}
		fprintf(log_file, "rtcm.time = %lld, obs.time = %lld, interval = %lld\n", rtcm.time.time, obs.time.time, obs.time.time - last_time > 1 ? obs.time.time - last_time : 0);
		last_time = obs.time.time;
		rtcm_buffer_cur = 0;
	}
}

void Rtcm_Split::create_new_split_file()
{
	if (time_split_file)fclose(time_split_file); time_split_file = NULL;
	if (time_split_file == NULL) {
		char file_path[256] = { 0 };
		gtime_t gtime = { 0 };
		gtime.time = split_start_time;
		sprintf(file_path, "%s_%s.rtcm", base_file_name, time_name(gtime, 0));
		time_split_file = fopen(file_path, "wb");
	}
}

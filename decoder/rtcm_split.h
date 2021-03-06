#pragma once
#include "rtcm.h"
#include <stdio.h>
#include <map>

#define RTCM_BUFF_SIZE 1024*16

class Rtcm_Split
{
public:
	Rtcm_Split();
	~Rtcm_Split();

	void init();
	void set_base_file_name(char* file_name);
	void set_time_ref(uint32_t ref_time);
	void set_time_range(uint32_t start_time,uint32_t end_time);
	void set_time_silce(uint32_t time_silce);
	void close_files();
	void split_data(uint8_t data);
	void split_data_repeat(uint8_t data);
	void create_new_split_file();
	void create_repeat_split_file();
private:
	char base_file_name[256];
	rtcm_t rtcm;
	obs_t obs;
	nav_t nav;
	uint8_t rtcm_buffer[RTCM_BUFF_SIZE];
	uint32_t rtcm_buffer_cur;
	std::map<uint32_t, FILE*> files_map;
	FILE* nav_file;
	FILE* log_file;
	FILE* time_split_file;
	uint32_t split_index;
	time_t split_start_time;
	time_t time_ref;
	time_t time_range_start;
	time_t time_range_end;
	time_t last_time;
	time_t m_time_slice;
};


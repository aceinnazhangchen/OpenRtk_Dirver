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
	void close_files();
	void input_data(uint8_t data);
private:
	char base_file_name[256];
	rtcm_t rtcm;
	obs_t obs;
	nav_t nav;
	uint8_t rtcm_buffer[RTCM_BUFF_SIZE];
	uint32_t rtcm_buffer_cur;
	std::map<uint32_t, FILE*> files_map;
};


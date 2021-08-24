#include "rtcm_split.h"
#include <string.h>

Rtcm_Split::Rtcm_Split()
{
	init();
}

Rtcm_Split::~Rtcm_Split()
{
}

void Rtcm_Split::init()
{
	memset(base_file_name, 0, 256);
	memset(&rtcm, 0, sizeof(rtcm));
	memset(&obs, 0, sizeof(obs));
	memset(&nav, 0, sizeof(nav));
	memset(rtcm_buffer, 0, RTCM_BUFF_SIZE);
	rtcm_buffer_cur = 0;
	files_map.clear();
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
}

void Rtcm_Split::input_data(uint8_t data)
{
	int ret = 0;
	ret = input_rtcm3_data(&rtcm,data, &obs, &nav);
	if (is_complete_rtcm()) {
		int size = rtcm.len + 3;
		if (rtcm_buffer_cur + size < RTCM_BUFF_SIZE) {
			memcpy(&rtcm_buffer[rtcm_buffer_cur], rtcm.buff, size);
			rtcm_buffer_cur += size;
		}
	}
	if (ret && obs.staid) {
		std::map<uint32_t, FILE*>::iterator it;
		it = files_map.find(obs.staid);
		if (it == files_map.end()) {
			char file_path[256] = { 0 };
			sprintf(file_path, "%s_%d.rtcm", base_file_name, obs.staid);
			FILE* file = fopen(file_path,"wb");
			files_map[obs.staid] = file;
		}
		FILE* sta_file = files_map[obs.staid];
		fwrite(rtcm_buffer, 1, rtcm_buffer_cur, sta_file);
		rtcm_buffer_cur = 0;
	}
}
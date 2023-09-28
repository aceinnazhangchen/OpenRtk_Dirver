#include <string.h>
#include "decode_interface.h"
#include "common.h"
#include "openrtk_user.h"
#include "openrtk_inceptio.h"
#include "rtk330la_decoder.h"
#include "ins401.h"
#include "ins_save_parse.h"
#include "beidou.h"
#include "NPOS122_decoder.h"
#include "ins401c.h"
#include "rtk350la.h"

#ifndef  WIN32
#include<ctype.h>
inline char* _strlwr(char* str)
{
	char* orig = str;
	// process the string
	for (; *str != '\0 '; str++)
		*str = tolower(*str);
	return orig;
}
#endif // ! WIN32


void decode_openrtk330li_interface(char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file) {
		char dirname[256] = { 0 };
		int ret = 0;
		int64_t file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		OpenRTK330LI_Tool::set_output_user_file(1);
		createDirByFilePath(filename, dirname);
		OpenRTK330LI_Tool::set_base_user_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret = OpenRTK330LI_Tool::input_user_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		OpenRTK330LI_Tool::write_kml_files();
		OpenRTK330LI_Tool::close_user_all_log_file();
		fclose(file);
		printf("\nfinished\r\n");
	}
}

void decode_rtk330la_interface(char* filename, bool pruned)
{
	RTK330LA_Tool::Rtk330la_decoder* rtk330la_decoder = new RTK330LA_Tool::Rtk330la_decoder();
	FILE* file = fopen(filename, "rb");
	if (file && rtk330la_decoder) {
		char dirname[256] = { 0 };
		int ret = 0;
		int64_t file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		createDirByFilePath(filename, dirname);
		rtk330la_decoder->init();
		rtk330la_decoder->set_pruned(pruned);
		rtk330la_decoder->set_base_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret = rtk330la_decoder->input_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		rtk330la_decoder->finish();
		fclose(file);
		printf("\nfinished\r\n");
	}
	delete rtk330la_decoder;
}

void decode_rtk350la_interface(char* filename, bool pruned)
{
	RTK350LA_Tool::RTK350LA_decoder* rtk350la_decoder = new RTK350LA_Tool::RTK350LA_decoder();
	FILE* file = fopen(filename, "rb");
	if (file && rtk350la_decoder) {
		char dirname[256] = { 0 };
		int ret = 0;
		int64_t file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		createDirByFilePath(filename, dirname);
		rtk350la_decoder->init();
		rtk350la_decoder->set_pruned(pruned);
		rtk350la_decoder->set_base_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret = rtk350la_decoder->input_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		rtk350la_decoder->finish();
		fclose(file);
		printf("\nfinished\r\n");
	}
	delete rtk350la_decoder;
}


void decode_ins401_interface(char* filename, char* is_parse_dr, bool pruned)
{
	Ins401_Tool::Ins401_decoder* ins401_decoder = new Ins401_Tool::Ins401_decoder();
	FILE* file = fopen(filename, "rb");
	if ((strcmp(_strlwr(is_parse_dr), "false") == 0) && (strstr(_strlwr(filename), "ins_save") != NULL))
	{
		return;
	}
	if (file && ins401_decoder) {
		int ret = 0;
		int64_t file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		char dirname[256] = { 0 };
		createDirByFilePath(filename, dirname);
		ins401_decoder->init();
		ins401_decoder->set_pruned(pruned);
		ins401_decoder->set_base_file_name(dirname);		
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret = ins401_decoder->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		if (strstr(filename, "ins_save") != NULL)
		{
			ins401_decoder->ins_save_finish();
		}
		else
		{
			ins401_decoder->finish();
		}
		fclose(file);
		printf("\nfinished\r\n");
	}
	delete ins401_decoder;
}

void decode_beidou_interface(char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file) {
		char dirname[256] = { 0 };
		int ret = 0;
		int64_t file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		beidou_Tool::set_output_beidou_file(1);
		createDirByFilePath(filename, dirname);
		beidou_Tool::set_base_beidou_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret = beidou_Tool::input_beidou_raw(read_cache[i]);
                ret = beidou_Tool::input_unico_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		beidou_Tool::write_beidou_kml_files();
		beidou_Tool::close_beidou_all_log_file();
		fclose(file);
		printf("\nfinished\r\n");
	}
}

void decode_npos122_interface(char* filename) {
	FILE* file = fopen(filename, "rb");
	NPOS122_Tool::NPOS122_decoder* npos122_decoder = new NPOS122_Tool::NPOS122_decoder();
	if (file && npos122_decoder) {
		int8_t ret = 0;
		int8_t ret_nmea = 0;
		int64_t file_size = getFileSize(file);
		int64_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		char dirname[256] = { 0 };
		createDirByFilePath(filename, dirname);
		npos122_decoder->init();
		npos122_decoder->set_base_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret_nmea = npos122_decoder->parse_nmea(read_cache[i]);
				ret = npos122_decoder->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		npos122_decoder->finish();
		fclose(file);
		printf("\nfinished\r\n");
	}
}

void decode_ins401c_interface(char* filename){
	printf("file name = %s\r\n", filename);
	FILE* file = fopen(filename, "r");
	if (file) {
		int ret = 0;
		char dirname[256] = { 0 };
		int64_t file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		uint8_t read_cache[1024] = { 0 };
		createDirByFilePath(filename, dirname);
		ins401c_Tool::set_base_ins401c_file_name(dirname);

        while(fgets((char* )read_cache, 1024, file)) {
			readcount = strlen((char*)read_cache);
			read_size += readcount;
            ret = ins401c_Tool::input_ins401c_line(read_cache);
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
        printf("Process : %4.1f %%\r", 100.0);
		ins401c_Tool::write_ins401c_kml_files();
		fclose(file);
		printf("\nfinished\r\n");
    }
}
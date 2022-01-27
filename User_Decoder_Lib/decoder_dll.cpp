#include "common.h"
#include "openrtk_user.h"
#include "openrtk_inceptio.h"
#include "ins401.h"
#include "decoder_dll.h"
#include <string.h>
#include "ins_save_parse.h"
#include "beidou.h"

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

USERDECODERLIB_API void decode_openrtk_user(char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file) {
		char dirname[256] = { 0 };
		int ret = 0;
		int file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		set_output_user_file(1);
		createDirByFilePath(filename, dirname);
		set_base_user_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret = input_user_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		write_kml_files();
		close_user_all_log_file();
		fclose(file);
		printf("\nfinished\r\n");
	}
}

USERDECODERLIB_API void decode_openrtk_inceptio(char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file) {
		char dirname[256] = { 0 };
		int ret = 0;
		int file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		RTK330LA_Tool::set_output_inceptio_file(1);
		createDirByFilePath(filename, dirname);
		RTK330LA_Tool::set_base_inceptio_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (size_t i = 0; i < readcount; i++) {
				ret = RTK330LA_Tool::input_inceptio_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		RTK330LA_Tool::write_inceptio_kml_files();
		RTK330LA_Tool::close_inceptio_all_log_file();
		fclose(file);
		printf("\nfinished\r\n");
	}
}


USERDECODERLIB_API void decode_beidou(char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file) {
		char dirname[256] = { 0 };
		int ret = 0;
		int file_size = getFileSize(file);
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

USERDECODERLIB_API void decode_ins401(char* filename, char* is_parse_dr, uint32_t inskml_rate)
{
	Ins401_Tool::Ins401_decoder* ins401_decoder = new Ins401_Tool::Ins401_decoder();
	FILE* file = fopen(filename, "rb");
	if( (strcmp(_strlwr(is_parse_dr), "false") == 0) && (strstr(_strlwr(filename), "ins_save") != NULL ) )
	{
		return;
	}
	if (file && ins401_decoder) {
		int ret = 0;
		int file_size = getFileSize(file);
		size_t read_size = 0;
		size_t readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		char dirname[256] = { 0 };
		createDirByFilePath(filename, dirname);
		ins401_decoder->init();
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
		if(strstr(filename, "ins_save") != NULL)
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
}

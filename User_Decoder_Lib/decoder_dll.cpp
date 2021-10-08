#include "common.h"
#include "openrtk_user.h"
#include "openrtk_inceptio.h"
#include "ins401.h"
#include "decoder_dll.h"
#include <string.h>
#include "ins_save_parse.h"

USERDECODERLIB_API void decode_openrtk_user(char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file) {
		char dirname[256] = { 0 };
		int ret = 0;
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		set_output_user_file(1);
		createDirByFilePath(filename, dirname);
		set_base_user_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
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
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		set_output_inceptio_file(1);
		createDirByFilePath(filename, dirname);
		set_base_inceptio_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = input_inceptio_raw(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		write_inceptio_kml_files();
		close_inceptio_all_log_file();
		fclose(file);
		printf("\nfinished\r\n");
	}
}

USERDECODERLIB_API void decode_ins401(char* filename)
{
	char* parse_str;
	char ins_parse_save_file[500];
	Ins401::Ins401_decoder* ins401_decoder = new Ins401::Ins401_decoder();
	FILE* file = fopen(filename, "rb");
	if (file && ins401_decoder) {
		int ret = 0;
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		char dirname[256] = { 0 };
		createDirByFilePath(filename, dirname);
		ins401_decoder->init();
		ins401_decoder->set_base_file_name(dirname);
		if(strstr(filename, "ins_save") != NULL)
		{
			strcpy(ins_parse_save_file,filename);
			char* bin_ptr = strstr(ins_parse_save_file,".bin");
			memcpy(bin_ptr,".txt",4);
			FILE* file_save = fopen(ins_parse_save_file, "w+");
			
			while (!feof(file)) {
				readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
				read_size += readcount;
				parse_str = parse_ins_save_data(read_cache, readcount);
				double percent = (double)read_size / (double)file_size * 100;
				printf("%s\r\n",parse_str);
				fwrite(parse_str,1,strlen(parse_str),file_save);
				printf("Process : %4.1f %%\r", percent);
			}
		}
		else
		{
			while (!feof(file)) {
				readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
				read_size += readcount;
				for (int i = 0; i < readcount; i++) {
					ret = ins401_decoder->input_data(read_cache[i]);
				}
				double percent = (double)read_size / (double)file_size * 100;
				printf("Process : %4.1f %%\r", percent);
			}
		}
		ins401_decoder->finish();
		fclose(file);
		printf("\nfinished\r\n");
	}
}

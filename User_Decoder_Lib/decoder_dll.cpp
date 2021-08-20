#include <stdio.h>
#include <vector>
#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif
#include <string.h>
#include "openrtk_user.h"
#include "openrtk_inceptio.h"
#include "ins401.h"
#include "decoder_dll.h"

#define READ_CACHE_SIZE 4*1024

int getFileSize(FILE* file)
{
	fseek(file, 0L, SEEK_END);
	int file_size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	return file_size;
}

int makeDir(char* folderPath)
{
	int ret = -1;
#ifdef WIN32
	if (0 != _access(folderPath, 0))
	{
		ret = _mkdir(folderPath);
	}
#else
	if(-1 == access(folderPath, 0)){
		ret = mkdir(folderPath,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
	}
#endif
	return ret;
}

void createDirByFilePath(char* filename, char* dirname) {
	char basename[64] = { 0 };
	strncpy(dirname, filename, strlen(filename) - 4);
	char* p = NULL;
#ifdef WIN32
	p = strrchr(dirname, '\\');
#else
	p = strrchr(dirname, '/');
#endif
	strcpy(basename, p);
	strcat(dirname, "_d");
	makeDir(dirname);
	strcat(dirname, basename);
}

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
		std::vector<user_g1_t> gnss_list;
		std::vector<user_i1_t> ins_list;
		set_output_user_file(1);
		set_save_bin(1);
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
	}
}

USERDECODERLIB_API void decode_ins401(char* filename)
{
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
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				ret = ins401_decoder->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		ins401_decoder->finish();
		fclose(file);
	}
}

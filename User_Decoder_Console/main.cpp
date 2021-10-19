#include "common.h"
#include "openrtk_user.h"
#include "openrtk_inceptio.h"
#include "ins401.h"

void decode_openrtk_user(char* filename)
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
	}
}

void decode_openrtk_inceptio(char* filename)
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

void decode_ins401(char* filename)
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
		if(strstr(filename, "ins_save") != NULL)
		{
			ins401_decoder->ins_save_finish();
		}
		else
		{
			ins401_decoder->finish();
		}
		fclose(file);
	}
	delete ins401_decoder;
}

int main(int argc, char* argv[]) {
	if (argc > 2) {
		char* filename = argv[2];
		if (strcmp(argv[1], "-u") == 0) {
			decode_openrtk_user(filename);
		}
		else if (strcmp(argv[1], "-i") == 0) {
			decode_openrtk_inceptio(filename);
		}
		else if (strcmp(argv[1], "-ins401") == 0) {
			decode_ins401(filename);
		}
		else {
			printf("User_Decoder_Console.exe -u(user)|i(inceptio) <filename> \n");
		}
	}
	else {
		printf("User_Decoder_Console.exe -u(user)|i(inceptio) <filename> \n");
	}
	return 0;
}
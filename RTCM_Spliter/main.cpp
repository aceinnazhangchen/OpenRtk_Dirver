#include "common.h"
#include  "rtcm_split.h"

void createDirRTCMBase(char* filename, char* dirname) {
	char basename[64] = "rtcm_base";
	char* p = NULL;
#ifdef WIN32
	p = strrchr(filename, '\\');
#else
	p = strrchr(filename, '/');
#endif
	strncpy(dirname, filename, strlen(filename) - strlen(p));
	sprintf(dirname, "%s\\%s\\", dirname, basename);
	makeDir(dirname);
	strcat(dirname, basename);
}

void split_rtcm(char* filename) {
	Rtcm_Split* rtcm_split = new Rtcm_Split();
	FILE* file = fopen(filename, "rb");	
	if (file && rtcm_split) {
		int file_size = getFileSize(file);
		int read_size = 0;
		int readcount = 0;
		char read_cache[READ_CACHE_SIZE] = { 0 };
		char dirname[256] = { 0 };
		createDirRTCMBase(filename, dirname);
		rtcm_split->init();
		rtcm_split->set_base_file_name(dirname);
		while (!feof(file)) {
			readcount = fread(read_cache, sizeof(char), READ_CACHE_SIZE, file);
			read_size += readcount;
			for (int i = 0; i < readcount; i++) {
				rtcm_split->input_data(read_cache[i]);
			}
			double percent = (double)read_size / (double)file_size * 100;
			printf("Process : %4.1f %%\r", percent);
		}
		rtcm_split->close_files();
		fclose(file);
	}
	delete rtcm_split;
}

int main(int argc, char* argv[]) {
	if (argc > 1) {
		char* filename = argv[1];
		split_rtcm(filename);
	}
	else {
		printf("RTCM_Spliter.exe <filename> \n");
	}
	return 0;
}
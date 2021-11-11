#include "common.h"
#include <stdio.h>
#include <string.h>
#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

const char* NMEATypeList[MAX_NMEA_TYPES] = { "$GPGGA","$GNGGA", "$GPRMC", "$GNRMC", "$GPGSV", "$GLGSV", "$GAGSV", "$BDGSV", "$GPGSA", "$GLGSA", "$GAGSA", "$BDGSA", "$GPZDA", "$GNZDA", "$GPVTG", "$PASHR", "$GNINS" };

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
	if (-1 == access(folderPath, 0)) {
		ret = mkdir(folderPath, S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
	}
#endif
	return ret;
}

void createDirByFilePath(const char* filename, char* dirname) {
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

uint16_t calc_crc(uint8_t* buff, uint32_t nbyte) {
	uint16_t crc = 0x1D0F;
	uint32_t i, j;
	for (i = 0; i < nbyte; i++) {
		crc = crc ^ (buff[i] << 8);
		for (j = 0; j < 8u; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			}
			else {
				crc = crc << 1;
			}
		}
	}
	crc = crc & 0xffff;
	return crc;
}

const char * nmea_type(int index)
{
	return NMEATypeList[index];
}

int is_nmea_char(char c)
{
	if (c >= 'A' && c <= 'Z') {
		return 1;
	}
	else if (c >= '0' && c <= '9') {
		return 1;
	}
	else if (c == '$' ,c == '-' || c == ',' || c == '.' || c == '*') {
		return 1;
	}
	return 0;
}

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

//int getFileSize(FILE* file)
//{
//	fseek(file, 0L, SEEK_END);
//	int file_size = ftell(file);
//	fseek(file, 0L, SEEK_SET);
//	return file_size;
//}

int64_t getFileSize(FILE * file)
{
	int64_t file_size = 0;
#if defined(_WIN32) || defined(_WIN64)
#if _MSC_VER >= 1400
	_fseeki64(file, (int64_t)(0), SEEK_END);
	file_size = _ftelli64(file);
	_fseeki64(file, (int64_t)(0), SEEK_SET);
	return file_size;
#else
#error Visual Studio version is less than 8.0(VS 2005) !
#endif
#else
	fseeko(file, (int64_t)(0), SEEK_END);
	file_size = ftello(file);
	fseeko(file, (int64_t)(0), SEEK_SET);
	return file_size;
#endif
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
	char* p_dot = NULL;
	p_dot = strrchr(filename, '.');
	char* p_separator = NULL;
#ifdef WIN32
	p_separator = strrchr(filename, '\\');
#else
	p_separator = strrchr(filename, '/');
#endif
	if (!p_separator) return;
	size_t ext_len = 0;
	if (p_dot && p_dot > p_separator) {
		ext_len = strlen(p_dot);
	}
	strncpy(basename, p_separator, strlen(p_separator) - ext_len);
	strncpy(dirname, filename, strlen(filename) - ext_len);
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

#define CRC32_POLYNOMIAL 0xEDB88320L;
uint32_t crc32_value(int32_t i)
{
	int32_t j;
	uint32_t ulCRC;
	ulCRC = i;
	for (j = 8; j > 0; j--)
	{
		if (ulCRC & 1u) {
			ulCRC = (ulCRC >> 1u) ^ CRC32_POLYNOMIAL;
		}
		else {
			ulCRC >>= 1u;
		}
	}
	return ulCRC;
}
// Data block
// Number of bytes in the data block
uint32_t cal_crc_32(const uint8_t* buffer,uint32_t nbyte)
{
	uint32_t ulTemp1;
	uint32_t ulTemp2;
	uint32_t ulCRC = 0;
	while (nbyte-- != 0)
	{
		ulTemp1 = (ulCRC >> 8u) & 0x00FFFFFFL;
		ulTemp2 = crc32_value(((int32_t)ulCRC ^ *buffer++) & 0xffu);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return(ulCRC);
}
#ifndef _COMMON_H_
#define _COMMON_H_
#include <stdio.h>
#include <stdint.h>

#define USER_PREAMB 0x55
#define NEAM_HEAD 0x24

#define READ_CACHE_SIZE 4*1024
#define MAX_NMEA_TYPES 17

#ifdef __cplusplus
extern "C" {
#endif

	int getFileSize(FILE* file);
	int makeDir(char* folderPath);
	void createDirByFilePath(const char* filename, char* dirname);
	uint16_t calc_crc(uint8_t* buff, uint32_t nbyte);
	const char* nmea_type(int index);
	int is_nmea_char(char c);
#ifdef __cplusplus
}
#endif

#endif

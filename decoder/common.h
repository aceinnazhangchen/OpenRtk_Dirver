#ifndef _COMMON_H_
#define _COMMON_H_
#include <stdio.h>
#include <stdint.h>

#define USER_PREAMB 0x55
#define NEAM_HEAD 0x24

#define READ_CACHE_SIZE 4*1024
#define MAX_NMEA_TYPES 17
#define NMEA_HEADER_LEN 6

#define HOUR	(3600)
#define MINUTE  (60)

#define MAX_INT 2147483648.0

#define CRC32_LEN 4

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)
	typedef struct
	{
		uint16_t GPS_Week;
		uint32_t GPS_TimeOfWeek;
		float x_accel;
		float y_accel;
		float z_accel;
		float x_gyro;
		float y_gyro;
		float z_gyro;
	} imu_t;
#pragma pack(pop)

	int64_t getFileSize(FILE* file);
	int makeDir(char* folderPath);
	void createDirByFilePath(const char* filename, char* dirname);
	uint16_t calc_crc(uint8_t* buff, uint32_t nbyte);
	uint32_t cal_crc_32(const uint8_t * buffer, uint32_t nbyte);
	const char* nmea_type(int index);
	int is_nmea_char(char c);
#ifdef __cplusplus
}
#endif

#endif

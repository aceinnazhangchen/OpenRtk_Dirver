#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mixed_raw.h"
#include "common.h"
#include "rtcm.h"

#ifndef NEAM_HEAD
#define NEAM_HEAD 0x24 //'$'
#endif // !NEAM_HEAD
#define IMU_HEAD_1 0xD4
#define IMU_HEAD_2 0x34

#define ROV_HEAD_LEN 8
#define BAS_HEAD_LEN 7

#pragma pack(push, 1)

typedef struct {
	uint32_t buffer_len;
	uint8_t buffer[MAX_BUFFER_SIZE];
	uint8_t type;
	uint8_t rov_index;
	uint32_t data_len;
}aceinna_raw_t;

#pragma pack(pop)
static int output_aceinna_file = 0;
static aceinna_raw_t raw = { 0 };
static char aceinna_file_basename[256] = { 0 };
static FILE* aceinna_log_file = NULL;
static FILE* aceinna_rov1_file = NULL;
static FILE* aceinna_bas_file = NULL;
static FILE* aceinna_imu_file = NULL;
static int is_decoding = 0;
void set_aceinna_decoding(int decoding)
{
	is_decoding = decoding;
	memset(&raw, 0, sizeof(aceinna_raw_t));
}
int is_aceinna_decoding()
{
	return is_decoding;
}
extern void set_output_aceinna_file(int output) {
	output_aceinna_file = output;
}
extern void set_aceinna_file_basename(char* input_name) {
	strcpy(aceinna_file_basename, input_name);
}
extern void open_aceinna_log_file() {
	if (output_aceinna_file == 0)return;
	if (strlen(aceinna_file_basename) == 0) return;
	char file_name[256] = { 0 };
	if (aceinna_log_file == NULL) {
		sprintf(file_name, "%s.log", aceinna_file_basename);
		aceinna_log_file = fopen(file_name, "w");
	}
}
extern void close_aceinna_all_file() {
	if (aceinna_log_file)fclose(aceinna_log_file); aceinna_log_file = NULL;
	if (aceinna_rov1_file)fclose(aceinna_rov1_file); aceinna_rov1_file = NULL;
	if (aceinna_bas_file)fclose(aceinna_bas_file); aceinna_bas_file = NULL;
	if (aceinna_imu_file)fclose(aceinna_imu_file); aceinna_imu_file = NULL;
}
static void write_aceinna_rov1_file(uint8_t *buff, uint32_t len) {
	if (output_aceinna_file == 0)return;
	if (strlen(aceinna_file_basename) == 0) return;
	char file_name[256] = { 0 };
	if (aceinna_rov1_file == NULL) {
		sprintf(file_name, "%s_rov1.rtcm", aceinna_file_basename);
		aceinna_rov1_file = fopen(file_name, "wb");
	}
	if (aceinna_rov1_file) fwrite(buff, 1, len, aceinna_rov1_file);
}
static void write_aceinna_bas_file(uint8_t *buff, uint32_t len) {
	if (output_aceinna_file == 0)return;
	if (strlen(aceinna_file_basename) == 0) return;
	char file_name[256] = { 0 };
	if (aceinna_bas_file == NULL) {
		sprintf(file_name, "%s_bas.rtcm", aceinna_file_basename);
		aceinna_bas_file = fopen(file_name, "wb");
	}
	if (aceinna_bas_file) fwrite(buff, 1, len, aceinna_bas_file);
}
static void open_aceinna_imu_file() {
	if (output_aceinna_file == 0)return;
	if (strlen(aceinna_file_basename) == 0) return;
	char file_name[256] = { 0 };
	if (aceinna_imu_file == NULL) {
		sprintf(file_name, "%s_imu.csv", aceinna_file_basename);
		aceinna_imu_file = fopen(file_name, "w");
		if (aceinna_imu_file) fprintf(aceinna_imu_file, "GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");
	}
}

void decode_aceinna_imu(uint8_t* buff) {
	if (output_aceinna_file == 0)return;
	imu_t pak = { 0 };
	memcpy(&pak, buff, IMU_CONST_SIZE);
	if (aceinna_imu_file) fprintf(aceinna_imu_file, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", pak.GPS_Week, (double)pak.gps_millisecs / 1000.0,
		pak.x_accel, pak.y_accel, pak.z_accel, pak.x_gyro, pak.y_gyro, pak.z_gyro);
}

extern int input_aceinna_format_raw(uint8_t c, uint8_t* outbuff, uint32_t* outlen) {
	int ret = 0;
	int crc = 0;
	if (raw.buffer_len == 0) {
		if (c == NEAM_HEAD) {
			raw.buffer[raw.buffer_len++] = c;
		}
	}
	else {
		if (raw.buffer_len < ACEINNA_HEAD_SIZE) {
			raw.buffer[raw.buffer_len++] = c;
			if (raw.buffer_len == ACEINNA_HEAD_SIZE) {
				if (strncmp(ROV_FLAG, (char*)raw.buffer, ACEINNA_HEAD_SIZE) == 0) {
					raw.type = TYPE_ROV;
				}
				else if (strncmp(BAS_FLAG, (char*)raw.buffer, ACEINNA_HEAD_SIZE) == 0) {
					raw.type = TYPE_BAS;
				}
				else if (strncmp(IMU_FLAG, (char*)raw.buffer, ACEINNA_HEAD_SIZE) == 0) {
					raw.type = TYPE_IMU;
				}
				else {
					raw.type = 0;
				}
			}
		}
		else {
			if (raw.type == TYPE_ROV) {
				raw.buffer[raw.buffer_len++] = c;
				if (raw.buffer_len == ROV_HEAD_LEN) {
					char str_bin_len[4] = { 0 };
					raw.rov_index = raw.buffer[ACEINNA_HEAD_SIZE] - 48;
					memcpy(str_bin_len, raw.buffer + ROV_HEAD_LEN - 3, 3);
					raw.data_len = atoi(str_bin_len);
				}
				if (raw.data_len > 0 && raw.buffer_len == ROV_HEAD_LEN + raw.data_len) {
					//write rov
					write_aceinna_rov1_file(raw.buffer + ROV_HEAD_LEN, raw.data_len);
					ret = raw.type;
					crc = rtcm_getbitu(raw.buffer + ROV_HEAD_LEN, (raw.data_len - 3) * 8, 24);
					if (aceinna_log_file)fprintf(aceinna_log_file, "$ROV,%d,%03d, %d\n", raw.rov_index, raw.data_len,crc);
					if (outbuff && outlen) {
						memcpy(outbuff, raw.buffer, raw.buffer_len);
						*outlen = raw.buffer_len;
					}
					memset(&raw, 0, sizeof(aceinna_raw_t));
				}
			}
			else if (raw.type == TYPE_BAS)
			{
				raw.buffer[raw.buffer_len++] = c;
				if (raw.buffer_len == BAS_HEAD_LEN) {
					char str_bin_len[4] = { 0 };
					memcpy(str_bin_len, raw.buffer + BAS_HEAD_LEN - 3, 3);
					raw.data_len = atoi(str_bin_len);
				}
				if (raw.data_len > 0 && raw.buffer_len == BAS_HEAD_LEN + raw.data_len) {
					//write ref
					write_aceinna_bas_file(raw.buffer + BAS_HEAD_LEN, raw.data_len);
					ret = raw.type;
					crc = rtcm_getbitu(raw.buffer + BAS_HEAD_LEN, (raw.data_len - 3) * 8, 24);
					if (aceinna_log_file)fprintf(aceinna_log_file, "$BAS,%d,%03d, %d\n", 0, raw.data_len, crc);
					if (outbuff && outlen) {
						memcpy(outbuff, raw.buffer, raw.buffer_len);
						*outlen = raw.buffer_len;
					}
					memset(&raw, 0, sizeof(aceinna_raw_t));
				}
			}
			else if (raw.type == TYPE_IMU)
			{
				raw.buffer[raw.buffer_len++] = c;
				if (raw.buffer_len == ACEINNA_HEAD_SIZE + IMU_CONST_SIZE) {
					//write imu
					open_aceinna_imu_file();
					decode_aceinna_imu(raw.buffer + ACEINNA_HEAD_SIZE);
					ret = raw.type;
					if (outbuff && outlen) {
						memcpy(outbuff, raw.buffer, raw.buffer_len);
						*outlen = raw.buffer_len;
					}
					memset(&raw, 0, sizeof(aceinna_raw_t));
				}
			}
			else {
				memset(&raw, 0, sizeof(aceinna_raw_t));
			}
		}
	}
	return  ret;
}
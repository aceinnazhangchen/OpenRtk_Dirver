#include <stdio.h>
#include <string.h>
#include "imu_raw.h"
#include "rtcm.h"

typedef struct {
	uint32_t nbyte;
	uint8_t buff[64];
}imu_raw_t;

static imu_raw_t imu_raw = { 0 };
static user_s1_t pak = { 0 };
FILE* imu_out = NULL;

char base_imu_file_name[256] = { 0 };
extern void set_base_imu_file_name(char* file_name)
{
	strcpy(base_imu_file_name, file_name);
}

void close_imu_all_log_file()
{
	if (imu_out)fclose(imu_out); imu_out = NULL;
}

void write_imu_log_file(char* log) {
	if(strlen(base_imu_file_name) == 0) return;
	char file_name[256] = { 0 };
	if (imu_out == NULL) {
		sprintf(file_name, "%s_imu.csv", base_imu_file_name);
		imu_out = fopen(file_name, "w");
		if (imu_out) fprintf(imu_out, "GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");
	}
	if (imu_out) fprintf(imu_out, log);
}

void parse_imu_packet_payload(uint8_t* buff, uint32_t nbyte, char* out_msg) {

	uint8_t payload_lenth = buff[2];
	uint8_t* payload = buff + 3;
	size_t packet_size = sizeof(user_s1_t);
	if (out_msg == NULL) return;
	if (payload_lenth == packet_size) {
		memcpy(&pak, payload, packet_size);
		//gtime_t gtime = gpst2time(pak.GPS_Week, (double)pak.GPS_TimeOfWeek / 1000.0);
		sprintf(out_msg, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", pak.GPS_Week, (double)pak.GPS_TimeOfWeek / 1000.0,
			pak.x_accel, pak.y_accel, pak.z_accel, pak.x_gyro, pak.y_gyro, pak.z_gyro);
		write_imu_log_file(out_msg);
	}
}

extern int input_imu_raw(uint8_t c, char* out_msg) {
	int ret = 0;
	if (imu_raw.nbyte < 2) {
		if (imu_raw.nbyte == 0 && c == 's') {
			imu_raw.buff[imu_raw.nbyte++] = c;
		}
		else if (imu_raw.nbyte == 1 && c == '1') {
			imu_raw.buff[imu_raw.nbyte++] = c;
		}
		else {
			imu_raw.nbyte = 0;
		}
	}
	else {
		imu_raw.buff[imu_raw.nbyte++] = c;
		if (imu_raw.nbyte == imu_raw.buff[2] + 5) {//5 = [type1,type2,len] + [crc1,crc2]
			uint16_t packet_crc = 256 * imu_raw.buff[imu_raw.nbyte - 2] + imu_raw.buff[imu_raw.nbyte - 1];
			if (packet_crc == calc_crc(imu_raw.buff, imu_raw.nbyte - 2)) {
				parse_imu_packet_payload(imu_raw.buff, imu_raw.nbyte, out_msg);
				ret = 1;
			}
			imu_raw.nbyte = 0;
		}
	}
	return ret;
}

extern user_s1_t * getImuPak()
{
	return &pak;
}

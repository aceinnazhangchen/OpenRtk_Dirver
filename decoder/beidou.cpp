#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <string.h>
#include <math.h>
#include "common.h"
#include "rtklib_core.h" //R2D
#include "kml.h"
#include "beidou.h"

#define IMU_OUT_PROCESS
#define BEIDOU_HEAD 0x23
#define MAX_INT 2147483648.0
#define MAX_BEIDOU_TYPES		3

namespace beidou_Tool {
	const char* beidouTypeList[MAX_BEIDOU_TYPES] = {"#HEADINGA", "#BESTVELA", "#BESTPOSA"};
	const char* beidouPacketsTypeList[MAX_beidou_PACKET_TYPES] = { "s1","gN","iN","o1","hG" };
	static usrRaw beidou_raw = { 0 };
    static unicoRaw unico_raw = {0};
	static char beidou_output_msg[1024] = { 0 };
	static char beidou_output_process[1024] = { 0 };
	static beidou_s1_t beidou_pak_s1 = { 0 };
	static beidou_gN_t beidou_pak_gN;
	static beidou_iN_t beidou_pak_iN = { 0 };
	static beidou_o1_t beidou_pak_o1 = { 0 };
	static beidou_hG_t beidou_pak_hG = {0};

    static RANGEHB_t unico_pak_rangeh = {0};
    static BD2SEPHEM_t unico_pak_bd2eph = {0};
    static OBSVM_t unico_pak_obs = {0};

	static kml_gnss_t gnss_kml = { 0 };
	static kml_ins_t ins_kml = { 0 };

	static int output_beidou_file = 0;
	static FILE* f_log = NULL;
	static FILE* fnmea = NULL;
	static FILE* fs1 = NULL;
	static FILE* fs2 = NULL;
	static FILE* fgN = NULL;
	static FILE* fiN = NULL;
	static FILE* fhG = NULL;
	static FILE* fo1 = NULL;
	static FILE* f_process = NULL;
	static FILE* f_gnssposvel = NULL;
	static FILE* f_imu = NULL;
	static FILE* f_ins = NULL;
	static FILE* f_odo = NULL;
	static FILE* f_s1 = NULL;
	static FILE* f_heading = NULL;
	static char base_beidou_file_name[256] = { 0 };

	int crc_error_num = 0;
	double	last_GPS_TimeOfWeek = 0.0;

    const uint32_t aulCrcTable[256] =
    {
        0x00000000UL, 0x77073096UL, 0xee0e612cUL, 0x990951baUL, 0x076dc419UL, 0x706af48fUL,
        0xe963a535UL, 0x9e6495a3UL, 0x0edb8832UL, 0x79dcb8a4UL, 0xe0d5e91eUL, 0x97d2d988UL,
        0x09b64c2bUL, 0x7eb17cbdUL, 0xe7b82d07UL, 0x90bf1d91UL, 0x1db71064UL, 0x6ab020f2UL,
        0xf3b97148UL, 0x84be41deUL, 0x1adad47dUL, 0x6ddde4ebUL, 0xf4d4b551UL, 0x83d385c7UL,
        0x136c9856UL, 0x646ba8c0UL, 0xfd62f97aUL, 0x8a65c9ecUL, 0x14015c4fUL, 0x63066cd9UL,
        0xfa0f3d63UL, 0x8d080df5UL, 0x3b6e20c8UL, 0x4c69105eUL, 0xd56041e4UL, 0xa2677172UL,
        0x3c03e4d1UL, 0x4b04d447UL, 0xd20d85fdUL, 0xa50ab56bUL, 0x35b5a8faUL, 0x42b2986cUL,
        0xdbbbc9d6UL, 0xacbcf940UL, 0x32d86ce3UL, 0x45df5c75UL, 0xdcd60dcfUL, 0xabd13d59UL,
        0x26d930acUL, 0x51de003aUL, 0xc8d75180UL, 0xbfd06116UL, 0x21b4f4b5UL, 0x56b3c423UL,
        0xcfba9599UL, 0xb8bda50fUL, 0x2802b89eUL, 0x5f058808UL, 0xc60cd9b2UL, 0xb10be924UL,
        0x2f6f7c87UL, 0x58684c11UL, 0xc1611dabUL, 0xb6662d3dUL, 0x76dc4190UL, 0x01db7106UL,
        0x98d220bcUL, 0xefd5102aUL, 0x71b18589UL, 0x06b6b51fUL, 0x9fbfe4a5UL, 0xe8b8d433UL,
        0x7807c9a2UL, 0x0f00f934UL, 0x9609a88eUL, 0xe10e9818UL, 0x7f6a0dbbUL, 0x086d3d2dUL,
        0x91646c97UL, 0xe6635c01UL, 0x6b6b51f4UL, 0x1c6c6162UL, 0x856530d8UL, 0xf262004eUL,
        0x6c0695edUL, 0x1b01a57bUL, 0x8208f4c1UL, 0xf50fc457UL, 0x65b0d9c6UL, 0x12b7e950UL,
        0x8bbeb8eaUL, 0xfcb9887cUL, 0x62dd1ddfUL, 0x15da2d49UL, 0x8cd37cf3UL, 0xfbd44c65UL,
        0x4db26158UL, 0x3ab551ceUL, 0xa3bc0074UL, 0xd4bb30e2UL, 0x4adfa541UL, 0x3dd895d7UL,
        0xa4d1c46dUL, 0xd3d6f4fbUL, 0x4369e96aUL, 0x346ed9fcUL, 0xad678846UL, 0xda60b8d0UL,
        0x44042d73UL, 0x33031de5UL, 0xaa0a4c5fUL, 0xdd0d7cc9UL, 0x5005713cUL, 0x270241aaUL,
        0xbe0b1010UL, 0xc90c2086UL, 0x5768b525UL, 0x206f85b3UL, 0xb966d409UL, 0xce61e49fUL,
        0x5edef90eUL, 0x29d9c998UL, 0xb0d09822UL, 0xc7d7a8b4UL, 0x59b33d17UL, 0x2eb40d81UL,
        0xb7bd5c3bUL, 0xc0ba6cadUL, 0xedb88320UL, 0x9abfb3b6UL, 0x03b6e20cUL, 0x74b1d29aUL,
        0xead54739UL, 0x9dd277afUL, 0x04db2615UL, 0x73dc1683UL, 0xe3630b12UL, 0x94643b84UL,
        0x0d6d6a3eUL, 0x7a6a5aa8UL, 0xe40ecf0bUL, 0x9309ff9dUL, 0x0a00ae27UL,
        0x7d079eb1UL,
        0xf00f9344UL, 0x8708a3d2UL, 0x1e01f268UL, 0x6906c2feUL, 0xf762575dUL, 0x806567cbUL,
        0x196c3671UL, 0x6e6b06e7UL, 0xfed41b76UL, 0x89d32be0UL, 0x10da7a5aUL, 0x67dd4accUL,
        0xf9b9df6fUL, 0x8ebeeff9UL, 0x17b7be43UL, 0x60b08ed5UL, 0xd6d6a3e8UL, 0xa1d1937eUL,
        0x38d8c2c4UL, 0x4fdff252UL, 0xd1bb67f1UL, 0xa6bc5767UL, 0x3fb506ddUL, 0x48b2364bUL,
        0xd80d2bdaUL, 0xaf0a1b4cUL, 0x36034af6UL, 0x41047a60UL, 0xdf60efc3UL, 0xa867df55UL,
        0x316e8eefUL, 0x4669be79UL, 0xcb61b38cUL, 0xbc66831aUL, 0x256fd2a0UL, 0x5268e236UL,
        0xcc0c7795UL, 0xbb0b4703UL, 0x220216b9UL, 0x5505262fUL, 0xc5ba3bbeUL, 0xb2bd0b28UL,
        0x2bb45a92UL, 0x5cb36a04UL, 0xc2d7ffa7UL, 0xb5d0cf31UL, 0x2cd99e8bUL, 0x5bdeae1dUL,
        0x9b64c2b0UL, 0xec63f226UL, 0x756aa39cUL, 0x026d930aUL, 0x9c0906a9UL, 0xeb0e363fUL,
        0x72076785UL, 0x05005713UL, 0x95bf4a82UL, 0xe2b87a14UL, 0x7bb12baeUL, 0x0cb61b38UL,
        0x92d28e9bUL, 0xe5d5be0dUL, 0x7cdcefb7UL, 0x0bdbdf21UL, 0x86d3d2d4UL, 0xf1d4e242UL,
        0x68ddb3f8UL, 0x1fda836eUL, 0x81be16cdUL, 0xf6b9265bUL, 0x6fb077e1UL, 0x18b74777UL,
        0x88085ae6UL, 0xff0f6a70UL, 0x66063bcaUL, 0x11010b5cUL, 0x8f659effUL,
        0xf862ae69UL,
        0x616bffd3UL, 0x166ccf45UL, 0xa00ae278UL, 0xd70dd2eeUL, 0x4e048354UL, 0x3903b3c2UL,
        0xa7672661UL, 0xd06016f7UL, 0x4969474dUL, 0x3e6e77dbUL, 0xaed16a4aUL, 0xd9d65adcUL,
        0x40df0b66UL, 0x37d83bf0UL, 0xa9bcae53UL, 0xdebb9ec5UL, 0x47b2cf7fUL, 0x30b5ffe9UL,
        0xbdbdf21cUL, 0xcabac28aUL, 0x53b39330UL, 0x24b4a3a6UL, 0xbad03605UL, 0xcdd70693UL,
        0x54de5729UL, 0x23d967bfUL, 0xb3667a2eUL, 0xc4614ab8UL, 0x5d681b02UL, 0x2a6f2b94UL,
        0xb40bbe37UL, 0xc30c8ea1UL, 0x5a05df1bUL, 0x2d02ef8dUL
    };

	const char* beidou_type(int index)
	{
		return beidouTypeList[index];
	}

	extern void init_beidou_data() {
		crc_error_num = 0;
		last_GPS_TimeOfWeek = 0;
		memset(&beidou_raw, 0, sizeof(usrRaw));
		memset(&beidou_pak_s1, 0, sizeof(beidou_s1_t));
		memset(&beidou_pak_gN, 0, sizeof(beidou_gN_t));
		memset(&beidou_pak_iN, 0, sizeof(beidou_iN_t));
		memset(&beidou_pak_o1, 0, sizeof(beidou_o1_t));
		memset(&beidou_pak_hG, 0, sizeof(beidou_hG_t));
        memset(&unico_pak_rangeh, 0, sizeof(RANGEHB_t));
        memset(&unico_pak_bd2eph, 0, sizeof(BD2SEPHEM_t));
        memset(&unico_pak_obs, 0, sizeof(OBSVM_t));
		Kml_Generator::Instance()->init();
	}

	extern void set_output_beidou_file(int output) {
		output_beidou_file = output;
	}
	extern void set_base_beidou_file_name(char* file_name)
	{
		strcpy(base_beidou_file_name, file_name);
		init_beidou_data();

		if (strlen(base_beidou_file_name) == 0) return;
		char log_file_name[256] = { 0 };
		if (f_log == NULL) {
			sprintf(log_file_name, "%s.log", base_beidou_file_name);
			f_log = fopen(log_file_name, "w");
		}
	}

	extern void close_beidou_all_log_file() {
		if (f_log)fclose(f_log); f_log = NULL;
		if (fnmea)fclose(fnmea); fnmea = NULL;
		if (fs1)fclose(fs1); fs1 = NULL;
		if (fs2)fclose(fs2); fs2 = NULL;
		if (fgN)fclose(fgN); fgN = NULL;
		if (fiN)fclose(fiN); fiN = NULL;
		if (fo1)fclose(fo1); fo1 = NULL;
		if (fhG)fclose(fhG); fhG = NULL;

		if (f_process)fclose(f_process); f_process = NULL;
		if (f_gnssposvel)fclose(f_gnssposvel); f_gnssposvel = NULL;
		if (f_imu)fclose(f_imu); f_imu = NULL;
		if (f_ins)fclose(f_ins); f_ins = NULL;
		if (f_odo)fclose(f_odo); f_odo = NULL;
		if (f_s1)fclose(f_s1); f_s1 = NULL;
		if (f_heading)fclose(f_heading); f_heading = NULL;
	}

	void beidou_append_gnss_kml() {
		gnss_kml.gps_week = beidou_pak_gN.week;
		gnss_kml.gps_secs = beidou_pak_gN.timeOfWeek;
		gnss_kml.position_type = beidou_pak_gN.positionMode;
		gnss_kml.latitude = (double)beidou_pak_gN.latitude*180.0 / MAX_INT;
		gnss_kml.longitude = (double)beidou_pak_gN.longitude*180.0 / MAX_INT;
		gnss_kml.height = beidou_pak_gN.height;
		gnss_kml.north_vel = (float)beidou_pak_gN.north_vel / 100.0f;
		gnss_kml.east_vel = (float)beidou_pak_gN.east_vel / 100.0f;
		gnss_kml.up_vel = (float)beidou_pak_gN.up_vel / 100.0f;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}

	void beidou_append_ins_kml() {
		ins_kml.gps_week = beidou_pak_iN.week;
		ins_kml.gps_secs = beidou_pak_iN.timeOfWeek;
		ins_kml.ins_status = beidou_pak_iN.insStatus;
		ins_kml.ins_position_type = beidou_pak_iN.insPositionType;
		ins_kml.latitude = (double)beidou_pak_iN.latitude*180.0 / MAX_INT;
		ins_kml.longitude = (double)beidou_pak_iN.longitude*180.0 / MAX_INT;
		ins_kml.height = beidou_pak_iN.height;
		ins_kml.north_velocity = (float)beidou_pak_iN.north_vel / 100.0f;
		ins_kml.east_velocity = (float)beidou_pak_iN.east_vel / 100.0f;
		ins_kml.up_velocity = (float)beidou_pak_iN.up_vel / 100.0f;
		ins_kml.roll = (float)beidou_pak_iN.roll / 100.0f;
		ins_kml.pitch = (float)beidou_pak_iN.pitch / 100.0f;
		ins_kml.heading = (float)beidou_pak_iN.heading / 100.0f;
		// printf("test1: %f \r\n",ins_kml.gps_secs);
		Kml_Generator::Instance()->append_ins(ins_kml);
	}

	void write_beidou_log_file(int index, char* log) {
		if (strlen(base_beidou_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case 0:
		{
			if (fnmea == NULL) {
				sprintf(file_name, "%s_nmea", base_beidou_file_name);
				fnmea = fopen(file_name, "w");
			}
			if (fnmea) fprintf(fnmea, log);
		}
		break;
		case beidou_OUT_SCALED1:
		{
			if (fs1 == NULL) {
				sprintf(file_name, "%s_s1.csv", base_beidou_file_name);
				fs1 = fopen(file_name, "w");
				if (fs1) fprintf(fs1, "GPS_Week(),GPS_TimeofWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n");
			}
			if (fs1) fprintf(fs1, log);
		}
		break;
		case beidou_OUT_GNSS:
		{
			if (fgN == NULL) {
				sprintf(file_name, "%s_gN.csv", base_beidou_file_name);
				fgN = fopen(file_name, "w");
				if (fgN) fprintf(fgN, "GPS_Week(),GPS_TimeofWeek(s),positionMode(),latitude(deg),longitude(deg),height(m),numberOfSVs(),hdop(),vdop(),tdop(),diffage(),velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s),latitude_std(),longitude_std(),height_std()\n");
			}
			if (fgN) fprintf(fgN, log);
		}
		break;
		case beidou_OUT_INSPVA:
		{
			if (fiN == NULL) {
				sprintf(file_name, "%s_iN.csv", base_beidou_file_name);
				fiN = fopen(file_name, "w");
				if (fiN) fprintf(fiN, "GPS_Week(),GPS_TimeofWeek(s),insStatus(),insPositionType(),latitude(deg),longitude(deg),height(m),velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s),roll(deg),pitch(deg),heading(deg)\n");
			}
			if (fiN) fprintf(fiN, log);
		}
		break;
		case beidou_OUT_ODO:
		{
			if (fo1 == NULL) {
				sprintf(file_name, "%s_o1.csv", base_beidou_file_name);
				fo1 = fopen(file_name, "w");
				if (fo1) fprintf(fo1, "GPS_Week(),GPS_TimeOfWeek(s),mode(),speed(m/s),fwd(),wheel_tick()\n");
			}
			if (fo1) fprintf(fo1, log);
		}
		break;
		case beidou_OUT_HEADING:
		{
			if (fhG == NULL) {
				sprintf(file_name, "%s_hG.csv", base_beidou_file_name);
				fhG = fopen(file_name, "w");
				if (fhG) fprintf(fhG, "GPS_Week(),GPS_TimeOfWeek(s),length(m),heading(),pitch(),hdgstddev(),ptchstddev()\n");
			}
			if (fhG) fprintf(fhG, log);
		}
		break;
		}
	}

	void write_beidou_ex_file(int index, char* log) {
		if (strlen(base_beidou_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case beidou_OUT_SCALED1:
		{
			if (f_imu == NULL) {
				sprintf(file_name, "%s_imu.txt", base_beidou_file_name);
				f_imu = fopen(file_name, "w");
			}
			if (f_imu) fprintf(f_imu, log);
		}
		break;
		case beidou_OUT_GNSS:
		{
			if (f_gnssposvel == NULL) {
				sprintf(file_name, "%s_gnssposvel.txt", base_beidou_file_name);
				f_gnssposvel = fopen(file_name, "w");
			}
			if (f_gnssposvel) fprintf(f_gnssposvel, log);
		}
		break;
		case beidou_OUT_INSPVA:
		{
			if (f_ins == NULL) {
				sprintf(file_name, "%s_ins.txt", base_beidou_file_name);
				f_ins = fopen(file_name, "w");
				if (f_ins) fprintf(f_ins, 
					"GPS_Week(),GPS_TimeOfWeek(s)"
					",latitude(deg),longitude(deg),height(m)"
					",north_velocity(m/s),east_velocity(m/s),up_velocity(m/s)"
					",roll(deg),pitch(deg),heading(deg)"
					",ins_position_type(),ins_status()\n");
			}
			if (f_ins) fprintf(f_ins, log);
		}
		break;
		case beidou_OUT_ODO:
		{
			if (f_odo == NULL) {
				sprintf(file_name, "%s_odo.txt", base_beidou_file_name);
				f_odo = fopen(file_name, "w");
			}
			if (f_odo) fprintf(f_odo, log);
		}
		break;
		case beidou_OUT_HEADING:
		{
			if (f_heading == NULL) {
				sprintf(file_name, "%s_heading.txt", base_beidou_file_name);
				f_heading = fopen(file_name, "w");
			}
			if (f_heading) fprintf(f_heading, log);
		}
		break;
		}
	}

	void write_beidou_process_file(int index, int type, char* log) {
		if (strlen(base_beidou_file_name) == 0) return;
		char file_name[256] = { 0 };
		if (f_process == NULL) {
			sprintf(file_name, "%s_process", base_beidou_file_name);
			f_process = fopen(file_name, "w");
		}
		switch (index)
		{
		case beidou_OUT_SCALED1:
		{
			if (f_process) fprintf(f_process, "$GPIMU,%s", log);
		}
		break;
		case beidou_OUT_GNSS:
		{
			if (type == 0) {
				if (f_process) fprintf(f_process, "$GPGNSS,%s", log);
			}
			else if (type == 1) {
				if (f_process) fprintf(f_process, "$GPVEL,%s", log);
			}
		}
		break;
		case beidou_OUT_INSPVA:
		{
			if (f_process) fprintf(f_process, "$GPINS,%s", log);
		}
		break;
		case beidou_OUT_ODO:
		{
			if (f_process) fprintf(f_process, "$GPODO,%s", log);
		}
		break;
		case beidou_OUT_HEADING:
		{
			if (f_process) fprintf(f_process, "$GPHEADING,%s", log);
		}		
		}
	}

	void write_beidou_kml_files() {
		Kml_Generator::Instance()->open_files(base_beidou_file_name);
		Kml_Generator::Instance()->write_files();
		Kml_Generator::Instance()->close_files();
	}

	void write_beidou_bin_file(int index, uint8_t* buff, uint32_t nbyte) {
		if (strlen(base_beidou_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case beidou_OUT_SCALED1:
		{
			if (f_s1 == NULL) {
				sprintf(file_name, "%s_s1.bin", base_beidou_file_name);
				f_s1 = fopen(file_name, "wb");
			}
			if (f_s1) fwrite(buff, 1, nbyte, f_s1);
		}
		break;
		}
	}

	void save_beidou_s1_to_user_s1() {
		uint8_t buffer[128] = { 0 };
		buffer[0] = 's';
		buffer[1] = '1';
		beidou_s1_t user_s1 = { 0 };
		user_s1.week = beidou_pak_s1.week;
		user_s1.timeOfWeek = (uint32_t)(beidou_pak_s1.timeOfWeek * 1000);
		user_s1.accel_g[0] = beidou_pak_s1.accel_g[0];
		user_s1.accel_g[1] = beidou_pak_s1.accel_g[1];
		user_s1.accel_g[2] = beidou_pak_s1.accel_g[2];
		user_s1.rate_dps[0] = beidou_pak_s1.rate_dps[0];
		user_s1.rate_dps[1] = beidou_pak_s1.rate_dps[1];
		user_s1.rate_dps[2] = beidou_pak_s1.rate_dps[2];
		uint8_t len = sizeof(beidou_s1_t);
		buffer[2] = len;
		memcpy(buffer + 3, &user_s1, len);
		uint16_t packet_crc = calc_crc(buffer, 3 + len);
		buffer[3 + len] = (packet_crc >> 8) & 0xff;
		buffer[3 + len + 1] = packet_crc & 0xff;
		write_beidou_bin_file(beidou_OUT_SCALED1, buffer, len + 5);
	}

	void output_beidou_s1() {
		//csv
		sprintf(beidou_output_msg, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", beidou_pak_s1.week, beidou_pak_s1.timeOfWeek,
			beidou_pak_s1.accel_g[0], beidou_pak_s1.accel_g[1], beidou_pak_s1.accel_g[2], beidou_pak_s1.rate_dps[0], beidou_pak_s1.rate_dps[1], beidou_pak_s1.rate_dps[2]);
		write_beidou_log_file(beidou_raw.ntype, beidou_output_msg);
		
		sprintf(beidou_output_process, "%d,%11.4f,   ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", beidou_pak_s1.week, beidou_pak_s1.timeOfWeek,
			beidou_pak_s1.accel_g[0], beidou_pak_s1.accel_g[1], beidou_pak_s1.accel_g[2], beidou_pak_s1.rate_dps[0], beidou_pak_s1.rate_dps[1], beidou_pak_s1.rate_dps[2]);
		////txt
		//sprintf(beidou_output_msg, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", beidou_pak_s1.GPS_Week, beidou_pak_s1.GPS_TimeOfWeek,
		//	beidou_pak_s1.x_accel, beidou_pak_s1.y_accel, beidou_pak_s1.z_accel, beidou_pak_s1.x_gyro, beidou_pak_s1.y_gyro, beidou_pak_s1.z_gyro);
		//write_beidou_ex_file(beidou_raw.ntype, beidou_output_msg);
		////process
#ifdef IMU_OUT_PROCESS
		write_beidou_process_file(beidou_raw.ntype, 0, beidou_output_process);
#endif
	}

	void output_beidou_gN() {
		if (beidou_pak_gN.positionMode == 0) {
			return;
		}
		double span_time = 0;
		if (last_GPS_TimeOfWeek != 0.0) {
			span_time = beidou_pak_gN.timeOfWeek - last_GPS_TimeOfWeek;
			if (span_time > 1) {
				fprintf(f_log, "%11.4f,%11.4f,%f \n", last_GPS_TimeOfWeek, beidou_pak_gN.timeOfWeek, span_time);
			}
		}
		float north_vel = (float)beidou_pak_gN.north_vel / 100.0f;
		float east_vel = (float)beidou_pak_gN.east_vel / 100.0f;
		float up_vel = (float)beidou_pak_gN.up_vel / 100.0f;
		float latitude_std = (float)beidou_pak_gN.latitude_std / 1000.0f;
		float longitude_std = (float)beidou_pak_gN.longitude_std / 1000.0f;
		float height_std = (float)beidou_pak_gN.height_std / 1000.0f;
		double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
		double track_over_ground = atan2(east_vel, north_vel) * R2D;
		float vdop = 0.0f;
		float tdop = 0.0f;
		//csv
		sprintf(beidou_output_msg, 
			"%d,%11.4f,%3d"
			",%14.9f,%14.9f,%10.4f"
			",%3d,%5.1f,%5.1f,%5.1f,%5.1f"
			",%5.1f,%5.1f,%5.1f"
			",%10.4f,%10.4f,%10.4f\n"
			,beidou_pak_gN.week, beidou_pak_gN.timeOfWeek, beidou_pak_gN.positionMode
			,(double)beidou_pak_gN.latitude*180.0 / MAX_INT, (double)beidou_pak_gN.longitude*180.0 / MAX_INT, beidou_pak_gN.height
			,beidou_pak_gN.numberOfSVs, beidou_pak_gN.hdop, vdop, tdop, (float)beidou_pak_gN.diffage
			,north_vel, east_vel, up_vel
			,latitude_std, longitude_std, height_std);
		write_beidou_log_file(beidou_raw.ntype, beidou_output_msg);
		//txt
		sprintf(beidou_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			beidou_pak_gN.week, beidou_pak_gN.timeOfWeek, beidou_pak_gN.latitude*180.0 / MAX_INT, beidou_pak_gN.longitude*180.0 / MAX_INT, beidou_pak_gN.height,
			latitude_std, longitude_std, height_std, beidou_pak_gN.positionMode, north_vel, east_vel, up_vel, track_over_ground);
		write_beidou_ex_file(beidou_raw.ntype, beidou_output_msg);
		//process $GPGNSS
		sprintf(beidou_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d\n", beidou_pak_gN.week, beidou_pak_gN.timeOfWeek,
			(double)beidou_pak_gN.latitude*180.0 / MAX_INT, (double)beidou_pak_gN.longitude*180.0 / MAX_INT, beidou_pak_gN.height, latitude_std, longitude_std, height_std, beidou_pak_gN.positionMode);
		write_beidou_process_file(beidou_raw.ntype, 0, beidou_output_msg);
		//process $GPVEL
		sprintf(beidou_output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n", beidou_pak_gN.week, beidou_pak_gN.timeOfWeek, horizontal_speed, track_over_ground, up_vel);
		write_beidou_process_file(beidou_raw.ntype, 1, beidou_output_msg);
		//kml
		beidou_append_gnss_kml();

		last_GPS_TimeOfWeek = beidou_pak_gN.timeOfWeek;
	}

	void output_beidou_iN() {
		//csv
		sprintf(beidou_output_msg, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f\n", beidou_pak_iN.week, beidou_pak_iN.timeOfWeek,
			beidou_pak_iN.insStatus, beidou_pak_iN.insPositionType,
			(double)beidou_pak_iN.latitude*180.0 / MAX_INT, (double)beidou_pak_iN.longitude*180.0 / MAX_INT, beidou_pak_iN.height,
			(float)beidou_pak_iN.north_vel / 100.0, (float)beidou_pak_iN.east_vel / 100.0, (float)beidou_pak_iN.up_vel / 100.0,
			(float)beidou_pak_iN.roll / 100.0, (float)beidou_pak_iN.pitch / 100.0, (float)beidou_pak_iN.heading / 100.0);
		write_beidou_log_file(beidou_raw.ntype, beidou_output_msg);
		uint32_t GPS_TimeOfWeek = (uint32_t)(beidou_pak_iN.timeOfWeek * 100) * 10;
		if (GPS_TimeOfWeek % 100 == 0) {
			//txt
			sprintf(beidou_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n", beidou_pak_iN.week, beidou_pak_iN.timeOfWeek,
				(double)beidou_pak_iN.latitude*180.0 / MAX_INT, (double)beidou_pak_iN.longitude*180.0 / MAX_INT, beidou_pak_iN.height,
				(float)beidou_pak_iN.north_vel / 100.0, (float)beidou_pak_iN.east_vel / 100.0, (float)beidou_pak_iN.up_vel / 100.0,
				(float)beidou_pak_iN.roll / 100.0, (float)beidou_pak_iN.pitch / 100.0, (float)beidou_pak_iN.heading / 100.0, beidou_pak_iN.insPositionType, beidou_pak_iN.insStatus);
			write_beidou_ex_file(beidou_raw.ntype, beidou_output_msg);
			//process
			sprintf(beidou_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d\n", beidou_pak_iN.week, beidou_pak_iN.timeOfWeek,
				(double)beidou_pak_iN.latitude*180.0 / MAX_INT, (double)beidou_pak_iN.longitude*180.0 / MAX_INT, beidou_pak_iN.height,
				(float)beidou_pak_iN.north_vel / 100.0, (float)beidou_pak_iN.east_vel / 100.0, (float)beidou_pak_iN.up_vel / 100.0,
				(float)beidou_pak_iN.roll / 100.0, (float)beidou_pak_iN.pitch / 100.0, (float)beidou_pak_iN.heading / 100.0, beidou_pak_iN.insPositionType);
			write_beidou_process_file(beidou_raw.ntype, 0, beidou_output_msg);
		}
		//kml
		beidou_append_ins_kml();
	}

	void output_beidou_o1() {
		//csv
		sprintf(beidou_output_msg, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", beidou_pak_o1.gps_week, (double)beidou_pak_o1.gps_millisecs / 1000.0, beidou_pak_o1.mode,
			beidou_pak_o1.speed, beidou_pak_o1.fwd, beidou_pak_o1.wheel_tick);
		write_beidou_log_file(beidou_raw.ntype, beidou_output_msg);
		//txt
		write_beidou_ex_file(beidou_raw.ntype, beidou_output_msg);
		//process
		write_beidou_process_file(beidou_raw.ntype, 0, beidou_output_msg);
	}

	void output_beidou_hG()
	{
		sprintf(beidou_output_msg, "%d,%11.4f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f\n",beidou_pak_hG.gps_week, (double)beidou_pak_hG.gps_millisecs / 1000.0,
			beidou_pak_hG.length, beidou_pak_hG.heading, beidou_pak_hG.pitch, beidou_pak_hG.hdgstddev, beidou_pak_hG.ptchstddev);
		write_beidou_log_file(beidou_raw.ntype, beidou_output_msg);
		//txt
		write_beidou_ex_file(beidou_raw.ntype, beidou_output_msg);
		//process
		write_beidou_process_file(beidou_raw.ntype, 0, beidou_output_msg);
	}

    void out_unico_rangeh()
    {
        // unico_pak_rangeh
    }

    void out_unico_bd2eph()
    {
        ;
    }
	void parse_beidou_packet_payload(uint8_t* buff, uint32_t nbyte) {
		uint8_t payload_lenth = buff[2];
		char packet_type[4] = { 0 };
		uint8_t* payload = buff + 3;
		char log_str[1024] = { 0 };
		memcpy(packet_type, buff, 2);
		if (strcmp(packet_type, "s1") == 0) {
			beidou_raw.ntype = beidou_OUT_SCALED1;
			if (payload_lenth == sizeof(beidou_s1_t)) {
				memcpy(&beidou_pak_s1, payload, sizeof(beidou_s1_t));
				output_beidou_s1();
				save_beidou_s1_to_user_s1();
			}
		}
		if (strcmp(packet_type, "gN") == 0) {
			beidou_raw.ntype = beidou_OUT_GNSS;
			if (payload_lenth == sizeof(beidou_gN_t)) {
				memcpy(&beidou_pak_gN, payload, payload_lenth);
				output_beidou_gN();
			}
		}
		else if (strcmp(packet_type, "iN") == 0) {
			beidou_raw.ntype = beidou_OUT_INSPVA;
			if (payload_lenth == sizeof(beidou_iN_t)) {
				memcpy(&beidou_pak_iN, payload, sizeof(beidou_iN_t));
				output_beidou_iN();
			}
		}
		else if (strcmp(packet_type, "o1") == 0) {
			beidou_raw.ntype = beidou_OUT_ODO;
			size_t psize = sizeof(beidou_o1_t);
			if (payload_lenth == sizeof(beidou_o1_t)) {
				memcpy(&beidou_pak_o1, payload, sizeof(beidou_o1_t));
				output_beidou_o1();
			}
		}
		else if(strcmp(packet_type, "hG") == 0)
		{
			beidou_raw.ntype = beidou_OUT_HEADING;
			size_t psize = sizeof(beidou_hG_t);
			if (payload_lenth == sizeof(beidou_hG_t)) {
				memcpy(&beidou_pak_hG, payload, sizeof(beidou_hG_t));
				output_beidou_hG();
			}			
		}
	}

	int parse_beidou_nmea(uint8_t data) {				//TODO:
		if (beidou_raw.nmea_flag == 0) {
			if ( (NEAM_HEAD == data) || (BEIDOU_HEAD == data) ) {
				beidou_raw.nmea_flag = 1;
				beidou_raw.nmeabyte = 0;
				beidou_raw.nmea[beidou_raw.nmeabyte++] = data;
			}
		}
		else if (beidou_raw.nmea_flag == 1) {
			beidou_raw.nmea[beidou_raw.nmeabyte++] = data;
			if (beidou_raw.nmeabyte == 6) {
				int i = 0;
				char NMEA[8] = { 0 };
				memcpy(NMEA, beidou_raw.nmea, 6);
				for (i = 0; i < MAX_NMEA_TYPES; i++) {
					if (strcmp(NMEA, nmea_type(i)) == 0) {
						beidou_raw.nmea_flag = 2;
						break;
					}
				}
				if (beidou_raw.nmea_flag != 2) {
					// beidou_raw.nmea_flag = 0;
				}
			}
			else if(beidou_raw.nmeabyte == 9)
			{
				int i = 0;
				char BEIDOU[10] = { 0 };
				memcpy(BEIDOU, beidou_raw.nmea, 9);
				for (i = 0; i < MAX_BEIDOU_TYPES; i++) {
					if (strcmp(BEIDOU, beidou_type(i)) == 0) {
						beidou_raw.nmea_flag = 2;
						break;
					}
				}
				if (beidou_raw.nmea_flag != 2) {
					beidou_raw.nmea_flag = 0;
				}
			}
		}
		else if (beidou_raw.nmea_flag == 2) {
			if (is_nmea_char(data)) {
				beidou_raw.nmea[beidou_raw.nmeabyte++] = data;
			}
			else {
				beidou_raw.nmea[beidou_raw.nmeabyte++] = 0x0A;
				beidou_raw.nmea[beidou_raw.nmeabyte++] = 0;
				beidou_raw.nmea_flag = 0;
				if (output_beidou_file) {
					write_beidou_log_file(0, (char*)beidou_raw.nmea);
				}
				return 2;
			}
		}
		return 0;
	}

	int input_beidou_raw(uint8_t data)
	{
		int ret = 0;
		if (beidou_raw.flag == 0) {
			beidou_raw.header[beidou_raw.header_len++] = data;
			if (beidou_raw.header_len == 1) {
				if (beidou_raw.header[0] != USER_PREAMB) {
					beidou_raw.header_len = 0;
				}
			}
			if (beidou_raw.header_len == 2) {
				if (beidou_raw.header[1] != USER_PREAMB) {
					beidou_raw.header_len = 0;
				}
			}
			if (beidou_raw.header_len == 4) {
				int i = 0;
				for (i = 0; i < MAX_beidou_PACKET_TYPES; i++) {
					const char* packetType = beidouPacketsTypeList[i];
					if (packetType[0] == beidou_raw.header[2] && packetType[1] == beidou_raw.header[3]) {
						beidou_raw.flag = 1;
						beidou_raw.buff[beidou_raw.nbyte++] = packetType[0];
						beidou_raw.buff[beidou_raw.nbyte++] = packetType[1];
						break;
					}
				}
				beidou_raw.header_len = 0;
			}
			return parse_beidou_nmea(data);
		}
		else {
			beidou_raw.buff[beidou_raw.nbyte++] = data;
			if (beidou_raw.nbyte == beidou_raw.buff[2] + 5) { //5 = [type1,type2,len] + [crc1,crc2]
				uint16_t packet_crc = 256 * beidou_raw.buff[beidou_raw.nbyte - 2] + beidou_raw.buff[beidou_raw.nbyte - 1];
				uint16_t cal_crc = calc_crc(beidou_raw.buff, beidou_raw.nbyte - 2);
				if (packet_crc == cal_crc) {
					parse_beidou_packet_payload(beidou_raw.buff, beidou_raw.nbyte);
					ret = 1;
				}
				else {
					crc_error_num++;
					fprintf(f_log, "type=%c%c,crc=0x%04x:0x%04x,size=%d\n", beidou_raw.buff[0], beidou_raw.buff[1], packet_crc, cal_crc, beidou_raw.nbyte);
				}
				beidou_raw.flag = 0;
				beidou_raw.nbyte = 0;
			}
		}
		return ret;
	}

    uint32_t CalculateCRC32(uint8_t *szBuf, uint32_t iSize)
    {
        int iIndex;
        uint32_t ulCRC = 0;
        for (iIndex=0; iIndex<iSize; iIndex++)
        {
            ulCRC = aulCrcTable[(ulCRC ^ szBuf[iIndex]) & 0xff] ^ (ulCRC >> 8);
        }
        return ulCRC;
    }

#define SYNC1 0xAA
#define SYNC2 0x44
#define SYNC3_1 0x12
#define SYNC3_2 0xB5
	int input_unico_raw(uint8_t data)
	{
		int ret = 0;
		if (unico_raw.flag == 0) {
			unico_raw.header[unico_raw.header_len++] = data;
			if (unico_raw.header_len == 1) {
				if (unico_raw.header[0] != SYNC1) {
					unico_raw.header_len = 0;
				}
			}
			if (unico_raw.header_len == 2) {
				if (unico_raw.header[1] != SYNC2) {
					unico_raw.header_len = 0;
				}
			}
			if (unico_raw.header_len == 3) {
				if ( (unico_raw.header[2] != SYNC3_1) && ((unico_raw.header[2] != SYNC3_2)) ) {
					unico_raw.header_len = 0;
				}
                else
                {
                    unico_raw.header_type = (unico_raw.header[2] == SYNC3_1 ? header_old:header_new);
                    unico_raw.header_to_read = (unico_raw.header[2] == SYNC3_1 ? sizeof(UnicoHeader_old_t):sizeof(UnicoHeader_t));
                    printf("unico_raw.header_type = %d, unico_raw.header_to_read = %d\r\n", unico_raw.header_type, unico_raw.header_to_read);
                }
			}
			if (unico_raw.header_len == unico_raw.header_to_read ) 
            {
                if(unico_raw.header_type == header_new)
                {
                    unico_raw.ntype = ((UnicoHeader_t*)(unico_raw.header))->message_id;
                    unico_raw.data_len = ((UnicoHeader_t*)(unico_raw.header))->message_length;
                }
                else{
                    unico_raw.ntype = ((UnicoHeader_old_t*)(unico_raw.header))->message_id;
                    unico_raw.data_len = ((UnicoHeader_old_t*)(unico_raw.header))->message_length;
                }

#if 0
				printf("sync1 = %02x, sync2 = %02x, sync3 = %02x, cpu_idle = %d, message_id = %d, message_length = %d, time_ref = %d, time_status = %d, week = %d, time_of_week = %d, version = %d, leap_sec = %d, delay_ms = %d\r\n", \
					test->sync1, \
					test->sync2, \
					test->sync3, \
					test->cpu_idle, \
					test->message_id, \
					test->message_length, \
					test->time_ref, \
					test->time_status, \
					test->week, \
					test->time_of_week, \
					test->version, \
					test->leap_sec, \
					test->delay_ms  \
				);
#endif
				// unico_raw.header_len = 0;
                unico_raw.flag = 1;
			}
		}
		else {
			unico_raw.buff[unico_raw.nbyte++] = data;
            if(unico_raw.nbyte == 2)
            {
                switch(unico_raw.ntype)
                {
                    case 6005:
                        unico_pak_rangeh.satellite_num = (uint16_t(unico_raw.buff[1]) << 8) + (unico_raw.buff[0]) ;
                        break;
                    case 1047:
                        break;
                    case 13:
                        unico_pak_obs.obs_num = (uint16_t(unico_raw.buff[1]) << 8) + (unico_raw.buff[0]) ;
                        break;
                    default:
                        unico_raw.nbyte = 0;
                        unico_raw.header_len = 0;
                        unico_raw.flag = 0;
                        unico_raw.header_len = 0;
                        unico_raw.ntype = 0;
                        break;
                }
            }
            // printf("unico_raw.nbyte = %d, unico_raw.data_len = %d\r\n", unico_raw.nbyte, unico_raw.data_len);
			if (unico_raw.nbyte == unico_raw.data_len + 4)
            { 
				uint32_t packet_crc = ((uint32_t)(unico_raw.buff[unico_raw.nbyte - 1]) << 24) + \
                ((uint32_t)(unico_raw.buff[unico_raw.nbyte - 2]) << 16) +\
                ((uint32_t)(unico_raw.buff[unico_raw.nbyte - 3]) << 8) +\
                ((uint32_t)(unico_raw.buff[unico_raw.nbyte - 4]));

                uint32_t data_len = unico_raw.data_len;
                uint8_t* data_to_check_crc = (uint8_t*)malloc(sizeof(char) * (data_len + unico_raw.header_len));
                memcpy(data_to_check_crc, unico_raw.header, unico_raw.header_len);
                memcpy(data_to_check_crc + unico_raw.header_len, unico_raw.buff, data_len);

                uint32_t data_len_to_check_crc = data_len + unico_raw.header_len;
	            uint32_t cal_crc = CalculateCRC32(data_to_check_crc, (data_len + unico_raw.header_len));

                printf("packet_crc = %10x, cal_crc = %10x, unico_raw.nbyte = %d, data_len_to_check_crc = %d, unico_pak_obs.obs_num = %d, crc_data_len = %d\r\n", packet_crc, cal_crc, unico_raw.nbyte, data_len_to_check_crc, unico_pak_obs.obs_num, (data_len + unico_raw.header_len));
				if (packet_crc == cal_crc) 
                {
                    switch(unico_raw.ntype)
                    {
                        case 6005:
                            unico_pak_rangeh.data = (RangehData_t **)calloc(unico_pak_rangeh.satellite_num, sizeof(RangehData_t *));
                            for(uint32_t i = 0; i < unico_pak_rangeh.satellite_num; i ++)
                            {
                                unico_pak_rangeh.data[i] = (RangehData_t *)malloc(sizeof(RangehData_t));
                                memcpy(unico_pak_rangeh.data[i], (RangehData_t*)(unico_raw.buff + 4 + sizeof(RangehData_t) * i), sizeof(RangehData_t));
                                printf("%d, prn = %d, locktime = %f, psr = %lf, dopp = %f\r\n", i, unico_pak_rangeh.data[i]->prn, unico_pak_rangeh.data[i]->locktime, unico_pak_rangeh.data[i]->psr, unico_pak_rangeh.data[i]->dopp);
                            }
                            for(uint32_t i = 0; i < unico_pak_rangeh.satellite_num; i ++)
                            {
                                if(unico_pak_rangeh.data[i] != NULL)
                                {
                                    free(unico_pak_rangeh.data[i]);
                                    unico_pak_rangeh.data[i] = NULL;
                                }
                            }

                            free(unico_pak_rangeh.data);
                            unico_pak_rangeh.data = NULL;
                            break;
                        case 1047:
#if 0
							printf("1047 data len = %d\r\n", unico_raw.data_len);
                            memcpy(&unico_pak_bd2eph.prn, unico_raw.buff, unico_raw.data_len);
                            printf("prn = %d, tow = %f, week = %d, n = %f\r\n", unico_pak_bd2eph.prn, unico_pak_bd2eph.tow, unico_pak_bd2eph.week, unico_pak_bd2eph.n);
#endif
                            break;
                        case 13:
                            unico_pak_obs.data = (obsVM_data_t **)calloc(unico_pak_obs.obs_num, sizeof(obsVM_data_t *));
                            if((unico_pak_obs.obs_num > 0) && (unico_pak_obs.obs_num < 200))
                            {
                                // printf("unico_raw.data_len = %d, sizeof(obsVM_data_t) = %d\r\n", unico_raw.data_len, sizeof(obsVM_data_t));
                                for(uint32_t i = 0; i < unico_pak_obs.obs_num; i ++)
                                {
                                    unico_pak_obs.data[i] = (obsVM_data_t *)malloc(sizeof(obsVM_data_t));
                                    memcpy(unico_pak_obs.data[i], (obsVM_data_t*)(unico_raw.buff + 4 + sizeof(obsVM_data_t) * i), sizeof(obsVM_data_t));
                                }
                                for(uint32_t i = 0; i < unico_pak_obs.obs_num; i ++)
                                {
                                    if(unico_pak_obs.data[i] != NULL)
                                    {
                                        free(unico_pak_obs.data[i]);
                                        unico_pak_obs.data[i] = NULL;
                                    }
                                }

                                free(unico_pak_obs.data);
                                unico_pak_obs.data = NULL;
                            }
                            break;
                        default:
                            break;
                    }

					ret = 1;
				}
				else {
					crc_error_num++;
                    printf("crc error\r\n");
				}
				unico_raw.nbyte = 0;
                unico_raw.header_len = 0;
                unico_raw.flag = 0;
                unico_raw.ntype = 0;
			}
		}
		return ret;
	}


}
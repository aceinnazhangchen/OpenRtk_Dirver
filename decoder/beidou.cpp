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
	static char beidou_output_msg[1024] = { 0 };
	static char beidou_output_process[1024] = { 0 };
	static beidou_s1_t beidou_pak_s1 = { 0 };
	static beidou_gN_t beidou_pak_gN;
	static beidou_iN_t beidou_pak_iN = { 0 };
	static beidou_o1_t beidou_pak_o1 = { 0 };
	static beidou_hG_t beidou_pak_hG = {0};

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
		Kml_Generator::Instance()->init();
		Kml_Generator::Instance()->set_kml_frequency(100);
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
				sprintf(file_name, "%s-nmea", base_beidou_file_name);
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
				sprintf(file_name, "%s-imu.txt", base_beidou_file_name);
				f_imu = fopen(file_name, "w");
			}
			if (f_imu) fprintf(f_imu, log);
		}
		break;
		case beidou_OUT_GNSS:
		{
			if (f_gnssposvel == NULL) {
				sprintf(file_name, "%s-gnssposvel.txt", base_beidou_file_name);
				f_gnssposvel = fopen(file_name, "w");
			}
			if (f_gnssposvel) fprintf(f_gnssposvel, log);
		}
		break;
		case beidou_OUT_INSPVA:
		{
			if (f_ins == NULL) {
				sprintf(file_name, "%s-ins.txt", base_beidou_file_name);
				f_ins = fopen(file_name, "w");
			}
			if (f_ins) fprintf(f_ins, log);
		}
		break;
		case beidou_OUT_ODO:
		{
			if (f_odo == NULL) {
				sprintf(file_name, "%s-odo.txt", base_beidou_file_name);
				f_odo = fopen(file_name, "w");
			}
			if (f_odo) fprintf(f_odo, log);
		}
		break;
		case beidou_OUT_HEADING:
		{
			if (f_heading == NULL) {
				sprintf(file_name, "%s-heading.txt", base_beidou_file_name);
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
			sprintf(file_name, "%s-process", base_beidou_file_name);
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
		
		sprintf(beidou_output_process, "%d,%11.4f,   %14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", beidou_pak_s1.week, beidou_pak_s1.timeOfWeek,
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
		sprintf(beidou_output_msg, "%d,%11.4f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f\n",beidou_pak_hG.gps_week, (double)beidou_pak_hG.gps_millisecs ,
			beidou_pak_hG.length, beidou_pak_hG.heading, beidou_pak_hG.pitch, beidou_pak_hG.hdgstddev, beidou_pak_hG.ptchstddev);
		write_beidou_log_file(beidou_raw.ntype, beidou_output_msg);
		//txt
		write_beidou_ex_file(beidou_raw.ntype, beidou_output_msg);
		//process
		write_beidou_process_file(beidou_raw.ntype, 0, beidou_output_msg);
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

}
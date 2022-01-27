#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <string.h>
#include <math.h>
#include "common.h"
#include "openrtk_inceptio.h"
#include "rtklib_core.h" //R2D
#include "kml.h"


#define VERSION_EARLY		0
#define VERSION_24_01_21	1
namespace RTK330LA_Tool {
	const char* inceptioPacketsTypeList[MAX_INCEPTIO_PACKET_TYPES] = { "s1","s2","gN","iN","d1","d2","sT","o1","fM","rt","sP" };

	static int data_version = 0;
	static usrRaw inceptio_raw = { 0 };
	static char inceptio_output_msg[1024] = { 0 };
	static inceptio_s1_t inceptio_pak_s1 = { 0 };
	static inceptio_s1_t inceptio_pak_s2 = { 0 };
	static inceptio_gN_early_t inceptio_pak_gN_early;
	static inceptio_gN_t inceptio_pak_gN;
	static inceptio_iN_t inceptio_pak_iN = { 0 };
	static inceptio_d1_t inceptio_pak_d1 = { 0 };
	static inceptio_d2_t inceptio_pak_d2 = { 0 };
	static inceptio_sT_t inceptio_pak_sT = { 0 };
	static inceptio_o1_t inceptio_pak_o1 = { 0 };

	static kml_gnss_t gnss_kml = { 0 };
	static kml_ins_t ins_kml = { 0 };

	static int output_inceptio_file = 0;
	static FILE* f_log = NULL;
	static FILE* fnmea = NULL;
	static FILE* fs1 = NULL;
	static FILE* fs2 = NULL;
	static FILE* fgN = NULL;
	static FILE* fiN = NULL;
	static FILE* fd1 = NULL;
	static FILE* fd2 = NULL;
	static FILE* fsT = NULL;
	static FILE* fo1 = NULL;
	static FILE* f_process = NULL;
	static FILE* f_gnssposvel = NULL;
	static FILE* f_imu = NULL;
	static FILE* f_ins = NULL;
	static FILE* f_odo = NULL;
	static FILE* fs1_b = NULL;
	static char base_inceptio_file_name[256] = { 0 };

	int crc_error_num = 0;
	double	last_GPS_TimeOfWeek = 0.0;

	extern void init_inceptio_data() {
		crc_error_num = 0;
		last_GPS_TimeOfWeek = 0;
		memset(&inceptio_raw, 0, sizeof(usrRaw));
		memset(&inceptio_pak_s1, 0, sizeof(inceptio_s1_t));
		memset(&inceptio_pak_s2, 0, sizeof(inceptio_s1_t));
		memset(&inceptio_pak_gN_early, 0, sizeof(inceptio_gN_early_t));
		memset(&inceptio_pak_gN, 0, sizeof(inceptio_gN_t));
		memset(&inceptio_pak_iN, 0, sizeof(inceptio_iN_t));
		memset(&inceptio_pak_d1, 0, sizeof(inceptio_d1_t));
		memset(&inceptio_pak_d2, 0, sizeof(inceptio_d2_t));
		memset(&inceptio_pak_sT, 0, sizeof(inceptio_sT_t));
		memset(&inceptio_pak_o1, 0, sizeof(inceptio_o1_t));
		Kml_Generator::Instance()->init();
	}

	extern void set_output_inceptio_file(int output) {
		output_inceptio_file = output;
	}
	extern void set_base_inceptio_file_name(char* file_name)
	{
		strcpy(base_inceptio_file_name, file_name);
		init_inceptio_data();

		if (strlen(base_inceptio_file_name) == 0) return;
		char log_file_name[256] = { 0 };
		if (f_log == NULL) {
			sprintf(log_file_name, "%s.log", base_inceptio_file_name);
			f_log = fopen(log_file_name, "w");
		}
	}

	extern void close_inceptio_all_log_file() {
		if (f_log)fclose(f_log); f_log = NULL;
		if (fnmea)fclose(fnmea); fnmea = NULL;
		if (fs1)fclose(fs1); fs1 = NULL;
		if (fs2)fclose(fs2); fs2 = NULL;
		if (fgN)fclose(fgN); fgN = NULL;
		if (fiN)fclose(fiN); fiN = NULL;
		if (fd1)fclose(fd1); fd1 = NULL;
		if (fd2)fclose(fd2); fd2 = NULL;
		if (fsT)fclose(fsT); fsT = NULL;
		if (fo1)fclose(fo1); fo1 = NULL;

		if (f_process)fclose(f_process); f_process = NULL;
		if (f_gnssposvel)fclose(f_gnssposvel); f_gnssposvel = NULL;
		if (f_imu)fclose(f_imu); f_imu = NULL;
		if (f_ins)fclose(f_ins); f_ins = NULL;
		if (f_odo)fclose(f_odo); f_odo = NULL;

		if (fs1_b)fclose(fs1_b); fs1_b = NULL;
	}

	void inceptio_append_early_gnss_kml() {
		gnss_kml.gps_week = inceptio_pak_gN_early.GPS_Week;
		gnss_kml.gps_secs = inceptio_pak_gN_early.GPS_TimeOfWeek;
		gnss_kml.position_type = inceptio_pak_gN_early.positionMode;
		gnss_kml.latitude = (double)inceptio_pak_gN_early.latitude*180.0 / MAX_INT;
		gnss_kml.longitude = (double)inceptio_pak_gN_early.longitude*180.0 / MAX_INT;
		gnss_kml.height = inceptio_pak_gN_early.height;
		gnss_kml.north_vel = (float)inceptio_pak_gN_early.velocityNorth / 100.0f;
		gnss_kml.east_vel = (float)inceptio_pak_gN_early.velocityEast / 100.0f;
		gnss_kml.up_vel = (float)inceptio_pak_gN_early.velocityUp / 100.0f;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}

	void inceptio_append_gnss_kml() {
		gnss_kml.gps_week = inceptio_pak_gN.GPS_Week;
		gnss_kml.gps_secs = inceptio_pak_gN.GPS_TimeOfWeek;
		gnss_kml.position_type = inceptio_pak_gN.positionMode;
		gnss_kml.latitude = (double)inceptio_pak_gN.latitude*180.0 / MAX_INT;
		gnss_kml.longitude = (double)inceptio_pak_gN.longitude*180.0 / MAX_INT;
		gnss_kml.height = inceptio_pak_gN.height;
		gnss_kml.north_vel = (float)inceptio_pak_gN.velocityNorth / 100.0f;
		gnss_kml.east_vel = (float)inceptio_pak_gN.velocityEast / 100.0f;
		gnss_kml.up_vel = (float)inceptio_pak_gN.velocityUp / 100.0f;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}

	void inceptio_append_ins_kml() {
		ins_kml.gps_week = inceptio_pak_iN.GPS_Week;
		ins_kml.gps_secs = inceptio_pak_iN.GPS_TimeOfWeek;
		ins_kml.ins_status = inceptio_pak_iN.insStatus;
		ins_kml.ins_position_type = inceptio_pak_iN.insPositionType;
		ins_kml.latitude = (double)inceptio_pak_iN.latitude*180.0 / MAX_INT;
		ins_kml.longitude = (double)inceptio_pak_iN.longitude*180.0 / MAX_INT;
		ins_kml.height = inceptio_pak_iN.height;
		ins_kml.north_velocity = (float)inceptio_pak_iN.velocityNorth / 100.0f;
		ins_kml.east_velocity = (float)inceptio_pak_iN.velocityEast / 100.0f;
		ins_kml.up_velocity = (float)inceptio_pak_iN.velocityUp / 100.0f;
		ins_kml.roll = (float)inceptio_pak_iN.roll / 100.0f;
		ins_kml.pitch = (float)inceptio_pak_iN.pitch / 100.0f;
		ins_kml.heading = (float)inceptio_pak_iN.heading / 100.0f;
		Kml_Generator::Instance()->append_ins(ins_kml);
	}

	void write_inceptio_log_file(int index, char* log) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case 0:
		{
			if (fnmea == NULL) {
				sprintf(file_name, "%s-nmea", base_inceptio_file_name);
				fnmea = fopen(file_name, "w");
			}
			if (fnmea) fprintf(fnmea, log);
		}
		break;
		case INCEPTIO_OUT_SCALED1:
		{
			if (fs1 == NULL) {
				sprintf(file_name, "%s_s1.csv", base_inceptio_file_name);
				fs1 = fopen(file_name, "w");
				if (fs1) fprintf(fs1, "GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n");
			}
			if (fs1) fprintf(fs1, log);
		}
		break;
		case INCEPTIO_OUT_SCALED2:
		{
			if (fs2 == NULL) {
				sprintf(file_name, "%s_s2.csv", base_inceptio_file_name);
				fs2 = fopen(file_name, "w");
				if (fs2) fprintf(fs2, "GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n");
			}
			if (fs2) fprintf(fs2, log);
		}
		break;
		case INCEPTIO_OUT_GNSS:
		{
			if (fgN == NULL) {
				sprintf(file_name, "%s_gN.csv", base_inceptio_file_name);
				fgN = fopen(file_name, "w");
				if (VERSION_EARLY == data_version) {
					if (fgN) fprintf(fgN, "GPS_Week(),GPS_TimeofWeek(s),positionMode(),latitude(deg),longitude(deg),height(m),numberOfSVs(),hdop(),diffage(),velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s),latitude_std(m),longitude_std(m),height_std(m)\n");
				}
				else if (VERSION_24_01_21 == data_version) {
					if (fgN) fprintf(fgN, "GPS_Week(),GPS_TimeofWeek(s),positionMode(),latitude(deg),longitude(deg),height(m),numberOfSVs(),hdop(),vdop(),tdop(),diffage(),velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s),latitude_std(m),longitude_std(m),height_std(m),pos_hor_pl(m),pos_ver_pl(m),pos_status(),vel_hor_pl(m/s),vel_ver_pl(m/s),vel_status()\n");
				}
			}
			if (fgN) fprintf(fgN, log);
		}
		break;
		case INCEPTIO_OUT_INSPVA:
		{
			if (fiN == NULL) {
				sprintf(file_name, "%s_iN.csv", base_inceptio_file_name);
				fiN = fopen(file_name, "w");
				if (fiN) fprintf(fiN, "GPS_Week(),GPS_TimeofWeek(s),insStatus(),insPositionType(),latitude(deg),longitude(deg),height(m),velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s),roll(deg),pitch(deg),heading(deg)\n");
			}
			if (fiN) fprintf(fiN, log);
		}
		break;
		case INCEPTIO_OUT_STD1:
		{
			if (fd1 == NULL) {
				sprintf(file_name, "%s_d1.csv", base_inceptio_file_name);
				fd1 = fopen(file_name, "w");
				if (fd1) fprintf(fd1, "GPS_Week(),GPS_TimeofWeek(s),latitude_std(),longitude_std(),height_std(),north_vel_std(),east_vel_std(),up_vel_std(),roll_std(),pitch_std(),heading_std()\n");
			}
			if (fd1) fprintf(fd1, log);
		}
		break;
		case INCEPTIO_OUT_STD2:
		{
			if (fd2 == NULL) {
				sprintf(file_name, "%s_d2.csv", base_inceptio_file_name);
				fd2 = fopen(file_name, "w");
				if (fd2) fprintf(fd2, "GPS_Week(),GPS_TimeofWeek(s),latitude_std(),longitude_std(),height_std(),north_vel_std(),east_vel_std(),up_vel_std()\n");
			}
			if (fd2) fprintf(fd2, log);
		}
		break;
		case INCEPTIO_OUT_STATUS:
		{
			if (fsT == NULL) {
				sprintf(file_name, "%s_sT.csv", base_inceptio_file_name);
				fsT = fopen(file_name, "w");
				if (fsT) fprintf(fsT, "GPS_Week(),GPS_TimeofWeek(s),year(),mouth(),day(),hour(),min(),sec(),imu_temp_status,imu_acce_status,imu_gyro_status,imu_sensor_status1,imu_sensor_status2,imu_sensor_status3,imu_overall_status\
,gnss_data_status,gnss_signal_status,power,MCU_status,pps_status,zupt_det,odo_used,odo_recv,imu_s1_state,imu_s2_state,imu_s3_state,time_valid,antenna_sensing,gnss_chipset,pust_check\
,imu_temperature(),mcu_temperature()\n");
			}
			if (fsT) fprintf(fsT, log);
		}
		break;
		case INCEPTIO_OUT_ODO:
		{
			if (fo1 == NULL) {
				sprintf(file_name, "%s_o1.csv", base_inceptio_file_name);
				fo1 = fopen(file_name, "w");
				if (fo1) fprintf(fo1, "GPS_Week(),GPS_TimeOfWeek(s),mode(),speed(m/s),fwd(),wheel_tick()\n");
			}
			if (fo1) fprintf(fo1, log);
		}
		break;
		}
	}

	void write_inceptio_ex_file(int index, char* log) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		//case INCEPTIO_OUT_SCALED1:
		case INCEPTIO_OUT_SCALED2:
		{
			if (f_imu == NULL) {
				sprintf(file_name, "%s-imu.txt", base_inceptio_file_name);
				f_imu = fopen(file_name, "w");
			}
			if (f_imu) fprintf(f_imu, log);
		}
		break;
		case INCEPTIO_OUT_GNSS:
		{
			if (f_gnssposvel == NULL) {
				sprintf(file_name, "%s-gnssposvel.txt", base_inceptio_file_name);
				f_gnssposvel = fopen(file_name, "w");
			}
			if (f_gnssposvel) fprintf(f_gnssposvel, log);
		}
		break;
		case INCEPTIO_OUT_INSPVA:
		{
			if (f_ins == NULL) {
				sprintf(file_name, "%s-ins.txt", base_inceptio_file_name);
				f_ins = fopen(file_name, "w");
			}
			if (f_ins) fprintf(f_ins, log);
		}
		break;
		case INCEPTIO_OUT_ODO:
		{
			if (f_odo == NULL) {
				sprintf(file_name, "%s-odo.txt", base_inceptio_file_name);
				f_odo = fopen(file_name, "w");
			}
			if (f_odo) fprintf(f_odo, log);
		}
		break;
		}
	}

	void write_inceptio_process_file(int index, int type, char* log) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		if (f_process == NULL) {
			sprintf(file_name, "%s-process", base_inceptio_file_name);
			f_process = fopen(file_name, "w");
		}
		switch (index)
		{
		case INCEPTIO_OUT_SCALED1:
		case INCEPTIO_OUT_SCALED2:
		{
			if (f_process) fprintf(f_process, "$GPIMU,%s", log);
		}
		break;
		case INCEPTIO_OUT_GNSS:
		{
			if (type == 0) {
				if (f_process) fprintf(f_process, "$GPGNSS,%s", log);
			}
			else if (type == 1) {
				if (f_process) fprintf(f_process, "$GPVEL,%s", log);
			}
		}
		break;
		case INCEPTIO_OUT_INSPVA:
		{
			if (f_process) fprintf(f_process, "$GPINS,%s", log);
		}
		break;
		case INCEPTIO_OUT_ODO:
		{
			if (f_process) fprintf(f_process, "$GPODO,%s", log);
		}
		case INCEPTIO_OUT_STD2:
		{
			//if (f_process) fprintf(f_process, "$GPGNSS,%s", log);
		}
		break;
		}
	}

	void write_inceptio_kml_files() {
		Kml_Generator::Instance()->open_files(base_inceptio_file_name);
		Kml_Generator::Instance()->write_files();
		Kml_Generator::Instance()->close_files();
	}

	void write_inceptio_bin_file(int index, uint8_t* buff, uint32_t nbyte) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case INCEPTIO_OUT_SCALED1:
		{
			if (fs1_b == NULL) {
				sprintf(file_name, "%s_s1.bin", base_inceptio_file_name);
				fs1_b = fopen(file_name, "wb");
			}
			if (fs1_b) fwrite(buff, 1, nbyte, fs1_b);
		}
		break;
		}
	}

	void save_inceptio_s1_to_user_s1() {
		uint8_t buffer[128] = { 0 };
		buffer[0] = 's';
		buffer[1] = '1';
		user_s1_t user_s1 = { 0 };
		user_s1.GPS_Week = inceptio_pak_s1.GPS_Week;
		user_s1.GPS_TimeOfWeek = (uint32_t)(inceptio_pak_s1.GPS_TimeOfWeek * 1000);
		user_s1.x_accel = inceptio_pak_s1.x_accel;
		user_s1.y_accel = inceptio_pak_s1.y_accel;
		user_s1.z_accel = inceptio_pak_s1.z_accel;
		user_s1.x_gyro = inceptio_pak_s1.x_gyro;
		user_s1.y_gyro = inceptio_pak_s1.y_gyro;
		user_s1.z_gyro = inceptio_pak_s1.z_gyro;
		uint8_t len = sizeof(user_s1_t);
		buffer[2] = len;
		memcpy(buffer + 3, &user_s1, len);
		uint16_t packet_crc = calc_crc(buffer, 3 + len);
		buffer[3 + len] = (packet_crc >> 8) & 0xff;
		buffer[3 + len + 1] = packet_crc & 0xff;
		write_inceptio_bin_file(INCEPTIO_OUT_SCALED1, buffer, len + 5);
	}

	void output_inceptio_s1() {
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", inceptio_pak_s1.GPS_Week, inceptio_pak_s1.GPS_TimeOfWeek,
			inceptio_pak_s1.x_accel, inceptio_pak_s1.y_accel, inceptio_pak_s1.z_accel, inceptio_pak_s1.x_gyro, inceptio_pak_s1.y_gyro, inceptio_pak_s1.z_gyro);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
		////txt
		//sprintf(inceptio_output_msg, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", inceptio_pak_s1.GPS_Week, inceptio_pak_s1.GPS_TimeOfWeek,
		//	inceptio_pak_s1.x_accel, inceptio_pak_s1.y_accel, inceptio_pak_s1.z_accel, inceptio_pak_s1.x_gyro, inceptio_pak_s1.y_gyro, inceptio_pak_s1.z_gyro);
		//write_inceptio_ex_file(inceptio_raw.ntype, inceptio_output_msg);
		////process
		//write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);
	}

	void output_inceptio_s2() {
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", inceptio_pak_s2.GPS_Week, inceptio_pak_s2.GPS_TimeOfWeek,
			inceptio_pak_s2.x_accel, inceptio_pak_s2.y_accel, inceptio_pak_s2.z_accel, inceptio_pak_s2.x_gyro, inceptio_pak_s2.y_gyro, inceptio_pak_s2.z_gyro);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
		//txt
		sprintf(inceptio_output_msg, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", inceptio_pak_s2.GPS_Week, inceptio_pak_s2.GPS_TimeOfWeek,
			inceptio_pak_s2.x_accel, inceptio_pak_s2.y_accel, inceptio_pak_s2.z_accel, inceptio_pak_s2.x_gyro, inceptio_pak_s2.y_gyro, inceptio_pak_s2.z_gyro);
		write_inceptio_ex_file(inceptio_raw.ntype, inceptio_output_msg);
		//process
		write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);

	}

	void output_inceptio_gN_early() {
		float north_vel = (float)inceptio_pak_gN_early.velocityNorth / 100.0f;
		float east_vel = (float)inceptio_pak_gN_early.velocityEast / 100.0f;
		float up_vel = (float)inceptio_pak_gN_early.velocityUp / 100.0f;
		float latitude_std = (float)inceptio_pak_gN_early.latitude_std / 1000.0f;
		float longitude_std = (float)inceptio_pak_gN_early.longitude_std / 1000.0f;
		float height_std = (float)inceptio_pak_gN_early.height_std / 1000.0f;
		double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
		double track_over_ground = atan2(east_vel, north_vel) * R2D;
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%3d,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n",
			inceptio_pak_gN_early.GPS_Week, inceptio_pak_gN_early.GPS_TimeOfWeek,
			inceptio_pak_gN_early.positionMode, (double)inceptio_pak_gN_early.latitude * 180.0 / MAX_INT, (double)inceptio_pak_gN_early.longitude * 180.0 / MAX_INT, inceptio_pak_gN_early.height,
			inceptio_pak_gN_early.numberOfSVs, inceptio_pak_gN_early.hdop, (float)inceptio_pak_gN_early.diffage,
			north_vel, east_vel, up_vel, latitude_std, longitude_std, height_std);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
		//txt
		sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			inceptio_pak_gN_early.GPS_Week, inceptio_pak_gN_early.GPS_TimeOfWeek, inceptio_pak_gN_early.latitude * 180.0 / MAX_INT, inceptio_pak_gN_early.longitude * 180.0 / MAX_INT, inceptio_pak_gN_early.height,
			latitude_std, longitude_std, height_std, inceptio_pak_gN_early.positionMode, north_vel, east_vel, up_vel, track_over_ground);
		write_inceptio_ex_file(inceptio_raw.ntype, inceptio_output_msg);
		//process $GPGNSS
		sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d\n", inceptio_pak_gN_early.GPS_Week, inceptio_pak_gN_early.GPS_TimeOfWeek,
			(double)inceptio_pak_gN_early.latitude * 180.0 / MAX_INT, (double)inceptio_pak_gN_early.longitude * 180.0 / MAX_INT, inceptio_pak_gN_early.height, latitude_std, longitude_std, height_std, inceptio_pak_gN_early.positionMode);
		write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);
		//process $GPVEL
		sprintf(inceptio_output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n", inceptio_pak_gN_early.GPS_Week, inceptio_pak_gN_early.GPS_TimeOfWeek, horizontal_speed, track_over_ground, up_vel);
		write_inceptio_process_file(inceptio_raw.ntype, 1, inceptio_output_msg);
		//kml
		//write_inceptio_gnss_kml_file(&inceptio_pak_gN_early);
		//gN_early_sol_list.push_back(inceptio_pak_gN_early);
		inceptio_append_early_gnss_kml();
	}

	void output_inceptio_gN() {
		double span_time = 0;
		if (last_GPS_TimeOfWeek != 0.0) {
			span_time = inceptio_pak_gN.GPS_TimeOfWeek - last_GPS_TimeOfWeek;
			if (span_time > 1) {
				fprintf(f_log, "%11.4f,%11.4f,%f \n", last_GPS_TimeOfWeek, inceptio_pak_gN.GPS_TimeOfWeek, span_time);
			}
		}
		float north_vel = (float)inceptio_pak_gN.velocityNorth / 100.0f;
		float east_vel = (float)inceptio_pak_gN.velocityEast / 100.0f;
		float up_vel = (float)inceptio_pak_gN.velocityUp / 100.0f;
		float latitude_std = (float)inceptio_pak_d2.latitude_std / 100.0f;
		float longitude_std = (float)inceptio_pak_d2.longitude_std / 100.0f;
		float height_std = (float)inceptio_pak_d2.height_std / 100.0f;
		float pos_hor_pl = (float)inceptio_pak_gN.pos_hor_pl / 1000.0f;
		float pos_ver_pl = (float)inceptio_pak_gN.pos_ver_pl / 1000.0f;
		float vel_hor_pl = (float)inceptio_pak_gN.vel_hor_pl / 1000.0f;
		float vel_ver_pl = (float)inceptio_pak_gN.vel_ver_pl / 1000.0f;
		double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
		double track_over_ground = atan2(east_vel, north_vel) * R2D;
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%3d,%5.1f,%5.1f,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%3d\n",
			inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek,
			inceptio_pak_gN.positionMode, (double)inceptio_pak_gN.latitude*180.0 / MAX_INT, (double)inceptio_pak_gN.longitude*180.0 / MAX_INT, inceptio_pak_gN.height,
			inceptio_pak_gN.numberOfSVs, inceptio_pak_gN.hdop, inceptio_pak_gN.vdop, inceptio_pak_gN.tdop, (float)inceptio_pak_gN.diffage,
			north_vel, east_vel, up_vel, latitude_std, longitude_std, height_std,
			pos_hor_pl, pos_ver_pl, inceptio_pak_gN.pos_status, vel_hor_pl, vel_ver_pl, inceptio_pak_gN.vel_status);
		write_inceptio_log_file(INCEPTIO_OUT_GNSS, inceptio_output_msg);
		//txt
		sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek, inceptio_pak_gN.latitude*180.0 / MAX_INT, inceptio_pak_gN.longitude*180.0 / MAX_INT, inceptio_pak_gN.height,
			latitude_std, longitude_std, height_std, inceptio_pak_gN.positionMode, north_vel, east_vel, up_vel, track_over_ground);
		write_inceptio_ex_file(INCEPTIO_OUT_GNSS, inceptio_output_msg);
		//process $GPGNSS
		sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d\n", inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek,
			(double)inceptio_pak_gN.latitude*180.0 / MAX_INT, (double)inceptio_pak_gN.longitude*180.0 / MAX_INT, inceptio_pak_gN.height, latitude_std, longitude_std, height_std, inceptio_pak_gN.positionMode);
		write_inceptio_process_file(INCEPTIO_OUT_GNSS, 0, inceptio_output_msg);
		//process $GPVEL
		sprintf(inceptio_output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n", inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek, horizontal_speed, track_over_ground, up_vel);
		write_inceptio_process_file(INCEPTIO_OUT_GNSS, 1, inceptio_output_msg);
		//kml
		inceptio_append_gnss_kml();

		last_GPS_TimeOfWeek = inceptio_pak_gN.GPS_TimeOfWeek;
	}

	void output_inceptio_iN() {
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f\n", inceptio_pak_iN.GPS_Week, inceptio_pak_iN.GPS_TimeOfWeek,
			inceptio_pak_iN.insStatus, inceptio_pak_iN.insPositionType,
			(double)inceptio_pak_iN.latitude*180.0 / MAX_INT, (double)inceptio_pak_iN.longitude*180.0 / MAX_INT, inceptio_pak_iN.height,
			(float)inceptio_pak_iN.velocityNorth / 100.0, (float)inceptio_pak_iN.velocityEast / 100.0, (float)inceptio_pak_iN.velocityUp / 100.0,
			(float)inceptio_pak_iN.roll / 100.0, (float)inceptio_pak_iN.pitch / 100.0, (float)inceptio_pak_iN.heading / 100.0);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
		uint32_t GPS_TimeOfWeek = (uint32_t)(inceptio_pak_iN.GPS_TimeOfWeek * 100) * 10;
		if (GPS_TimeOfWeek % 100 == 0) {
			//txt
			sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n", inceptio_pak_iN.GPS_Week, inceptio_pak_iN.GPS_TimeOfWeek,
				(double)inceptio_pak_iN.latitude*180.0 / MAX_INT, (double)inceptio_pak_iN.longitude*180.0 / MAX_INT, inceptio_pak_iN.height,
				(float)inceptio_pak_iN.velocityNorth / 100.0, (float)inceptio_pak_iN.velocityEast / 100.0, (float)inceptio_pak_iN.velocityUp / 100.0,
				(float)inceptio_pak_iN.roll / 100.0, (float)inceptio_pak_iN.pitch / 100.0, (float)inceptio_pak_iN.heading / 100.0, inceptio_pak_iN.insPositionType, inceptio_pak_iN.insStatus);
			write_inceptio_ex_file(inceptio_raw.ntype, inceptio_output_msg);
			//process
			sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d\n", inceptio_pak_iN.GPS_Week, inceptio_pak_iN.GPS_TimeOfWeek,
				(double)inceptio_pak_iN.latitude*180.0 / MAX_INT, (double)inceptio_pak_iN.longitude*180.0 / MAX_INT, inceptio_pak_iN.height,
				(float)inceptio_pak_iN.velocityNorth / 100.0, (float)inceptio_pak_iN.velocityEast / 100.0, (float)inceptio_pak_iN.velocityUp / 100.0,
				(float)inceptio_pak_iN.roll / 100.0, (float)inceptio_pak_iN.pitch / 100.0, (float)inceptio_pak_iN.heading / 100.0, inceptio_pak_iN.insPositionType);
			write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);
		}
		//kml
		inceptio_append_ins_kml();
	}

	void output_inceptio_d1() {
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n", inceptio_pak_d1.GPS_Week, inceptio_pak_d1.GPS_TimeOfWeek,
			(float)inceptio_pak_d1.latitude_std / 100.0, (float)inceptio_pak_d1.longitude_std / 100.0, (float)inceptio_pak_d1.height_std / 100.0,
			(float)inceptio_pak_d1.north_vel_std / 100.0, (float)inceptio_pak_d1.east_vel_std / 100.0, (float)inceptio_pak_d1.up_vel_std / 100.0,
			(float)inceptio_pak_d1.roll_std / 100.0, (float)inceptio_pak_d1.pitch_std / 100.0, (float)inceptio_pak_d1.heading_std / 100.0);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
	}

	void output_inceptio_d2() {
		//csv
		//sprintf(inceptio_output_msg, "%d,%11.4f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%3d,%3d\n", inceptio_pak_d2.GPS_Week, inceptio_pak_d2.GPS_TimeOfWeek,
		//	(float)inceptio_pak_d2.latitude_std / 100.0, (float)inceptio_pak_d2.longitude_std / 100.0, (float)inceptio_pak_d2.height_std / 100.0,
		//	(float)inceptio_pak_d2.north_vel_std / 100.0, (float)inceptio_pak_d2.east_vel_std / 100.0, (float)inceptio_pak_d2.up_vel_std / 100.0,
		//	inceptio_pak_d2.hor_pos_pl, inceptio_pak_d2.ver_pos_pl, inceptio_pak_d2.hor_vel_pl, inceptio_pak_d2.ver_vel_pl, inceptio_pak_d2.pos_integrity_status, inceptio_pak_d2.vel_integrity_status);
		sprintf(inceptio_output_msg, "%d,%11.4f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n", inceptio_pak_d2.GPS_Week, inceptio_pak_d2.GPS_TimeOfWeek,
			(float)inceptio_pak_d2.latitude_std / 100.0, (float)inceptio_pak_d2.longitude_std / 100.0, (float)inceptio_pak_d2.height_std / 100.0,
			(float)inceptio_pak_d2.north_vel_std / 100.0, (float)inceptio_pak_d2.east_vel_std / 100.0, (float)inceptio_pak_d2.up_vel_std / 100.0);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
		write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);
	}

	void output_inceptio_sT() {
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%5d,%5d,%5d,%5d,%5d,%5d, %3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d, %8.3f,%8.3f\n", inceptio_pak_sT.GPS_Week, inceptio_pak_sT.GPS_TimeOfWeek,
			inceptio_pak_sT.year, inceptio_pak_sT.mouth, inceptio_pak_sT.day, inceptio_pak_sT.hour, inceptio_pak_sT.min, inceptio_pak_sT.sec,
			inceptio_pak_sT.status_bit.imu_temp_status, inceptio_pak_sT.status_bit.imu_acce_status, inceptio_pak_sT.status_bit.imu_gyro_status,
			inceptio_pak_sT.status_bit.imu_sensor_status1, inceptio_pak_sT.status_bit.imu_sensor_status2, inceptio_pak_sT.status_bit.imu_sensor_status3, inceptio_pak_sT.status_bit.imu_overall_status,
			inceptio_pak_sT.status_bit.gnss_data_status, inceptio_pak_sT.status_bit.gnss_signal_status, inceptio_pak_sT.status_bit.power, inceptio_pak_sT.status_bit.MCU_status, inceptio_pak_sT.status_bit.pps_status,
			inceptio_pak_sT.status_bit.zupt_det, inceptio_pak_sT.status_bit.odo_used, inceptio_pak_sT.status_bit.odo_recv,
			inceptio_pak_sT.status_bit.imu_s1_state, inceptio_pak_sT.status_bit.imu_s2_state, inceptio_pak_sT.status_bit.imu_s3_state,
			inceptio_pak_sT.status_bit.time_valid, inceptio_pak_sT.status_bit.antenna_sensing, inceptio_pak_sT.status_bit.gnss_chipset, 
			inceptio_pak_sT.status_bit.pust_check,
			inceptio_pak_sT.imu_temperature, inceptio_pak_sT.mcu_temperature);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
	}

	void output_inceptio_o1() {
		//csv
		sprintf(inceptio_output_msg, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", inceptio_pak_o1.GPS_Week, (double)inceptio_pak_o1.GPS_TimeOfWeek / 1000.0, inceptio_pak_o1.mode,
			inceptio_pak_o1.speed, inceptio_pak_o1.fwd, inceptio_pak_o1.wheel_tick);
		write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
		//txt
		write_inceptio_ex_file(inceptio_raw.ntype, inceptio_output_msg);
		//process
		write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);
	}

	void parse_inceptio_packet_payload(uint8_t* buff, uint32_t nbyte) {
		uint8_t payload_lenth = buff[2];
		char packet_type[4] = { 0 };
		uint8_t* payload = buff + 3;
		char log_str[1024] = { 0 };
		memcpy(packet_type, buff, 2);
		if (strcmp(packet_type, "s1") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_SCALED1;
			if (payload_lenth == sizeof(inceptio_s1_t)) {
				memcpy(&inceptio_pak_s1, payload, sizeof(inceptio_s1_t));
				output_inceptio_s1();
				save_inceptio_s1_to_user_s1();
			}
		}
		if (strcmp(packet_type, "s2") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_SCALED2;
			if (payload_lenth == sizeof(inceptio_s1_t)) {
				memcpy(&inceptio_pak_s2, payload, sizeof(inceptio_s1_t));
				output_inceptio_s2();
			}
		}
		else if (strcmp(packet_type, "gN") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_GNSS;
			if (payload_lenth == sizeof(inceptio_gN_more_early_t) ||
				payload_lenth == sizeof(inceptio_gN_early_t)) {
				data_version = VERSION_EARLY;
				memcpy(&inceptio_pak_gN_early, payload, payload_lenth);
				output_inceptio_gN_early();
			}
			if (payload_lenth == sizeof(inceptio_gN_24_01_21_t) ||
				payload_lenth == sizeof(inceptio_gN_t)) {
				data_version = VERSION_24_01_21;
				memcpy(&inceptio_pak_gN, payload, payload_lenth);
				//output_inceptio_gN();
			}
		}
		else if (strcmp(packet_type, "iN") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_INSPVA;
			if (payload_lenth == sizeof(inceptio_iN_t)) {
				memcpy(&inceptio_pak_iN, payload, sizeof(inceptio_iN_t));
				output_inceptio_iN();
			}
		}
		else if (strcmp(packet_type, "d1") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_STD1;
			if (payload_lenth == sizeof(inceptio_d1_t)) {
				memcpy(&inceptio_pak_d1, payload, sizeof(inceptio_d1_t));
				output_inceptio_d1();
			}
		}
		else if (strcmp(packet_type, "d2") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_STD2;
			if (payload_lenth == sizeof(inceptio_d2_t)) {
				memcpy(&inceptio_pak_d2, payload, sizeof(inceptio_d2_t));
				output_inceptio_d2();
				output_inceptio_gN();
			}
		}
		else if (strcmp(packet_type, "sT") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_STATUS;
			if (payload_lenth == sizeof(inceptio_sT_t)) {
				memcpy(&inceptio_pak_sT, payload, sizeof(inceptio_sT_t));
				output_inceptio_sT();
			}
		}
		else if (strcmp(packet_type, "o1") == 0) {
			inceptio_raw.ntype = INCEPTIO_OUT_ODO;
			size_t psize = sizeof(inceptio_o1_t);
			if (payload_lenth == sizeof(inceptio_o1_t)) {
				memcpy(&inceptio_pak_o1, payload, sizeof(inceptio_o1_t));
				output_inceptio_o1();
			}
		}
	}

	int parse_inceptio_nmea(uint8_t data) {
		if (inceptio_raw.nmea_flag == 0) {
			if (NEAM_HEAD == data) {
				inceptio_raw.nmea_flag = 1;
				inceptio_raw.nmeabyte = 0;
				inceptio_raw.nmea[inceptio_raw.nmeabyte++] = data;
			}
		}
		else if (inceptio_raw.nmea_flag == 1) {
			inceptio_raw.nmea[inceptio_raw.nmeabyte++] = data;
			if (inceptio_raw.nmeabyte == 6) {
				int i = 0;
				char NMEA[8] = { 0 };
				memcpy(NMEA, inceptio_raw.nmea, 6);
				for (i = 0; i < MAX_NMEA_TYPES; i++) {
					if (strcmp(NMEA, nmea_type(i)) == 0) {
						inceptio_raw.nmea_flag = 2;
						break;
					}
				}
				if (inceptio_raw.nmea_flag != 2) {
					inceptio_raw.nmea_flag = 0;
				}
			}
		}
		else if (inceptio_raw.nmea_flag == 2) {
			if (is_nmea_char(data)) {
				inceptio_raw.nmea[inceptio_raw.nmeabyte++] = data;
			}
			else {
				inceptio_raw.nmea[inceptio_raw.nmeabyte++] = 0x0A;
				inceptio_raw.nmea[inceptio_raw.nmeabyte++] = 0;
				inceptio_raw.nmea_flag = 0;
				if (output_inceptio_file) {
					write_inceptio_log_file(0, (char*)inceptio_raw.nmea);
				}
				return 2;
			}
		}
		return 0;
	}

	int input_inceptio_raw(uint8_t data)
	{
		int ret = 0;
		if (inceptio_raw.flag == 0) {
			inceptio_raw.header[inceptio_raw.header_len++] = data;
			if (inceptio_raw.header_len == 1) {
				if (inceptio_raw.header[0] != USER_PREAMB) {
					inceptio_raw.header_len = 0;
				}
			}
			if (inceptio_raw.header_len == 2) {
				if (inceptio_raw.header[1] != USER_PREAMB) {
					inceptio_raw.header_len = 0;
				}
			}
			if (inceptio_raw.header_len == 4) {
				int i = 0;
				for (i = 0; i < MAX_INCEPTIO_PACKET_TYPES; i++) {
					const char* packetType = inceptioPacketsTypeList[i];
					if (packetType[0] == inceptio_raw.header[2] && packetType[1] == inceptio_raw.header[3]) {
						inceptio_raw.flag = 1;
						inceptio_raw.buff[inceptio_raw.nbyte++] = packetType[0];
						inceptio_raw.buff[inceptio_raw.nbyte++] = packetType[1];
						break;
					}
				}
				inceptio_raw.header_len = 0;
			}
			return parse_inceptio_nmea(data);
		}
		else {
			inceptio_raw.buff[inceptio_raw.nbyte++] = data;
			if (inceptio_raw.nbyte == inceptio_raw.buff[2] + 5) { //5 = [type1,type2,len] + [crc1,crc2]
				uint16_t packet_crc = 256 * inceptio_raw.buff[inceptio_raw.nbyte - 2] + inceptio_raw.buff[inceptio_raw.nbyte - 1];
				uint16_t cal_crc = calc_crc(inceptio_raw.buff, inceptio_raw.nbyte - 2);
				if (packet_crc == cal_crc) {
					parse_inceptio_packet_payload(inceptio_raw.buff, inceptio_raw.nbyte);
					ret = 1;
				}
				else {
					crc_error_num++;
					fprintf(f_log, "type=%c%c,crc=0x%04x:0x%04x,size=%d\n", inceptio_raw.buff[0], inceptio_raw.buff[1], packet_crc, cal_crc, inceptio_raw.nbyte);
				}
				inceptio_raw.flag = 0;
				inceptio_raw.nbyte = 0;
			}
		}
		return ret;
	}

	uint8_t get_current_type() {
		return inceptio_raw.ntype;
	}

	inceptio_gN_t * get_gnss_sol()
	{
		return &inceptio_pak_gN;
	}

	inceptio_s1_t * get_imu_raw()
	{
		return &inceptio_pak_s2;
	}

}
#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <string.h>
#include <math.h>
#include "common.h"
#include "openrtk_user.h"
#include "rtklib_core.h" //R2D
#include "kml.h"

namespace OpenRTK330LI_Tool {

	const char* userPacketsTypeList[MAX_PACKET_TYPES] = { "s1", "g1", "i1", "o1", "y1" };
	static FILE* fnmea = NULL;
	static FILE* fs1 = NULL;
	static FILE* fg1 = NULL;
	static FILE* fi1 = NULL;
	static FILE* fo1 = NULL;
	static FILE* fy1 = NULL;
	static FILE* f_process = NULL;
	static FILE* f_gnssposvel = NULL;
	static FILE* f_imu = NULL;
	static FILE* f_ins = NULL;
	static FILE* f_odo = NULL;
	static FILE* fs1_b = NULL;

	static usrRaw user_raw = { 0 };
	static char user_output_msg[1024] = { 0 };
	static int output_user_file = 0;
	static user_s1_t pak_s1 = { 0 };
	static user_g1_t pak_g1 = { 0 };
	static user_i1_t pak_i1 = { 0 };
	static user_o1_t pak_o1 = { 0 };
	static user_y1_t pak_y1 = { 0 };

	static kml_gnss_t gnss_kml = { 0 };
	static kml_ins_t ins_kml = { 0 };

	static int save_bin = 0;
	static char base_user_file_name[256] = { 0 };

	extern void init_user_data() {
		memset(&user_raw, 0, sizeof(usrRaw));
		memset(&pak_s1, 0, sizeof(pak_s1));
		memset(&pak_g1, 0, sizeof(pak_g1));
		memset(&pak_i1, 0, sizeof(pak_i1));
		memset(&pak_o1, 0, sizeof(pak_o1));
		memset(&pak_y1, 0, sizeof(pak_y1));
		Kml_Generator::Instance()->init();
	}

	extern void set_save_bin(int save) {
		save_bin = save;
	}
	extern void set_output_user_file(int output) {
		output_user_file = output;
	}
	extern void set_base_user_file_name(char* file_name)
	{
		strcpy(base_user_file_name, file_name);
		init_user_data();
	}

	extern void close_user_all_log_file() {
		if (fnmea)fclose(fnmea); fnmea = NULL;
		if (fs1)fclose(fs1); fs1 = NULL;
		if (fg1)fclose(fg1); fg1 = NULL;
		if (fi1)fclose(fi1); fi1 = NULL;
		if (fo1)fclose(fo1); fo1 = NULL;
		if (fy1)fclose(fy1); fy1 = NULL;

		if (f_process)fclose(f_process); f_process = NULL;
		if (f_gnssposvel)fclose(f_gnssposvel); f_gnssposvel = NULL;
		if (f_imu)fclose(f_imu); f_imu = NULL;
		if (f_ins)fclose(f_ins); f_ins = NULL;
		if (f_odo)fclose(f_odo); f_odo = NULL;

		if (fs1_b)fclose(fs1_b); fs1_b = NULL;
	}

	void append_gnss_kml() {
		gnss_kml.gps_week = pak_g1.GPS_Week;
		gnss_kml.gps_secs = (double)pak_g1.GPS_TimeOfWeek / 1000.0;
		gnss_kml.position_type = pak_g1.position_type;
		gnss_kml.latitude = pak_g1.latitude;
		gnss_kml.longitude = pak_g1.longitude;
		gnss_kml.height = pak_g1.height;
		gnss_kml.north_vel = pak_g1.north_vel;
		gnss_kml.east_vel = pak_g1.east_vel;
		gnss_kml.up_vel = pak_g1.up_vel;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}

	void append_ins_kml() {
		ins_kml.gps_week = pak_i1.GPS_Week;
		ins_kml.gps_secs = (double)pak_i1.GPS_TimeOfWeek / 1000.0;
		ins_kml.ins_status = pak_i1.ins_status;
		ins_kml.ins_position_type = pak_i1.ins_position_type;
		ins_kml.latitude = pak_i1.latitude;
		ins_kml.longitude = pak_i1.longitude;
		ins_kml.height = pak_i1.height;
		ins_kml.north_velocity = (float)pak_i1.north_velocity;
		ins_kml.east_velocity = (float)pak_i1.east_velocity;
		ins_kml.up_velocity = (float)pak_i1.up_velocity;
		ins_kml.roll = (float)pak_i1.roll;
		ins_kml.pitch = (float)pak_i1.pitch;
		ins_kml.heading = (float)pak_i1.heading;
		Kml_Generator::Instance()->append_ins(ins_kml);
	}

	void write_user_process_file(int index, int type, char* log) {
		if (strlen(base_user_file_name) == 0) return;
		char file_name[256] = { 0 };
		if (f_process == NULL) {
			sprintf(file_name, "%s_process", base_user_file_name);
			f_process = fopen(file_name, "w");
		}
		switch (index)
		{
		case USR_OUT_RAWIMU:
		{
			if (f_process) fprintf(f_process, "$GPIMU,%s", log);
		}
		break;
		case USR_OUT_BESTGNSS:
		{
			if (type == 0) {
				if (f_process) fprintf(f_process, "$GPGNSS,%s", log);
			}
			else if (type == 1) {
				if (f_process) fprintf(f_process, "$GPVEL,%s", log);
			}
		}
		break;
		case USR_OUT_INSPVAX:
		{
			if (f_process) fprintf(f_process, "$GPINS,%s", log);
		}
		break;
		}
	}

	void write_kml_files() {
		Kml_Generator::Instance()->open_files(base_user_file_name);
		Kml_Generator::Instance()->write_files();
		Kml_Generator::Instance()->close_files();
	}

	void write_user_ex_file(int index, char* log) {
		if (strlen(base_user_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case USR_OUT_RAWIMU:
		{
			if (f_imu == NULL) {
				sprintf(file_name, "%s-imu.txt", base_user_file_name);
				f_imu = fopen(file_name, "w");
			}
			if (f_imu) fprintf(f_imu, log);
		}
		break;
		case USR_OUT_BESTGNSS:
		{
			if (f_gnssposvel == NULL) {
				sprintf(file_name, "%s-gnssposvel.txt", base_user_file_name);
				f_gnssposvel = fopen(file_name, "w");
			}
			if (f_gnssposvel) fprintf(f_gnssposvel, log);
		}
		break;
		case USR_OUT_INSPVAX:
		{
			if (f_ins == NULL) {
				sprintf(file_name, "%s-ins.txt", base_user_file_name);
				f_ins = fopen(file_name, "w");
				printf("0x%p\n", f_ins);
			}
			if (f_ins) fprintf(f_ins, log);
		}
		break;
		case USR_OUT_ODO:
		{
			if (f_odo == NULL) {
				sprintf(file_name, "%s-odo.txt", base_user_file_name);
				f_odo = fopen(file_name, "w");
			}
			if (f_odo) fprintf(f_odo, log);
		}
		break;
		}
	}

	void write_user_log_file(int index, char* log) {
		if (strlen(base_user_file_name) == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case 0:
		{
			if (fnmea == NULL) {
				sprintf(file_name, "%s-nmea", base_user_file_name);
				fnmea = fopen(file_name, "w");
			}
			if (fnmea) fprintf(fnmea, log);
		}
		break;
		case USR_OUT_RAWIMU:
		{
			if (fs1 == NULL) {
				sprintf(file_name, "%s_s1.csv", base_user_file_name);
				fs1 = fopen(file_name, "w");
				if (fs1) fprintf(fs1, "GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");
			}
			if (fs1) fprintf(fs1, log);
		}
		break;
		case USR_OUT_BESTGNSS:
		{
			if (fg1 == NULL) {
				sprintf(file_name, "%s_g1.csv", base_user_file_name);
				fg1 = fopen(file_name, "w");
				if (fg1) fprintf(fg1, "GPS_Week(),GPS_TimeOfWeek(s),position_type(),latitude(deg),longitude(deg),height(m),latitude_standard_deviation(m),longitude_standard_deviation(m),height_standard_deviation(m),number_of_satellites(),number_of_satellites_in_solution(),hdop(),diffage(s),north_vel(m/s),east_vel(m/s),up_vel(m/s),north_vel_standard_deviation(m/s),east_vel_standard_deviation(m/s),up_vel_standard_deviation(m/s)\n");
			}
			if (fg1) fprintf(fg1, log);
		}
		break;
		case USR_OUT_INSPVAX:
		{
			if (fi1 == NULL) {
				sprintf(file_name, "%s_i1.csv", base_user_file_name);
				fi1 = fopen(file_name, "w");
				if (fi1) fprintf(fi1, "GPS_Week(),GPS_TimeOfWeek(s),ins_status(),ins_position_type(),latitude(deg),longitude(deg),height(m),north_velocity(m/s),east_velocity(m/s),up_velocity(m/s),roll(deg),pitch(deg),heading(deg),latitude_std(m),longitude_std(m),height_std(m),north_velocity_std(m/s),east_velocity_std(m/s),up_velocity_std(m/s),roll_std(deg),pitch_std(deg),heading_std(deg)\n");
			}
			if (fi1) fprintf(fi1, log);
		}
		break;
		case USR_OUT_ODO:
		{
			if (fo1 == NULL) {
				sprintf(file_name, "%s_o1.csv", base_user_file_name);
				fo1 = fopen(file_name, "w");
				if (fo1) fprintf(fo1, "GPS_Week(),GPS_TimeOfWeek(s),mode,speed,fwd,wheel_tick\n");
			}
			if (fo1) fprintf(fo1, log);
		}
		break;
		case USR_OUT_SATELLITES:
		{
			if (fy1 == NULL) {
				sprintf(file_name, "%s_y1.csv", base_user_file_name);
				fy1 = fopen(file_name, "w");
				if (fy1) fprintf(fy1, "GPS_Week(),GPS_TimeOfWeek(s),satelliteId(),systemId(),antennaId(),l1cn0(),l2cn0(),azimuth(deg),elevation(deg)\n");
			}
			if (fy1) fprintf(fy1, log);
		}
		break;
		}
	}

	void write_user_bin_file(int index, uint8_t* buff, uint32_t nbyte) {
		if (strlen(base_user_file_name) == 0) return;
		if (save_bin == 0) return;
		char file_name[256] = { 0 };
		switch (index)
		{
		case USR_OUT_RAWIMU:
		{
			if (fs1_b == NULL) {
				sprintf(file_name, "%s_s1.bin", base_user_file_name);
				fs1_b = fopen(file_name, "wb");
			}
			if (fs1_b) fwrite(buff, 1, nbyte, fs1_b);
		}
		break;
		}
	}

	void output_user_s1() {
		//csv
		sprintf(user_output_msg, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", pak_s1.GPS_Week, (double)pak_s1.GPS_TimeOfWeek / 1000.0,
			pak_s1.x_accel, pak_s1.y_accel, pak_s1.z_accel, pak_s1.x_gyro, pak_s1.y_gyro, pak_s1.z_gyro);
		write_user_log_file(USR_OUT_RAWIMU, user_output_msg);
		//txt
		sprintf(user_output_msg, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", pak_s1.GPS_Week, (double)pak_s1.GPS_TimeOfWeek / 1000.0,
			pak_s1.x_accel, pak_s1.y_accel, pak_s1.z_accel, pak_s1.x_gyro, pak_s1.y_gyro, pak_s1.z_gyro);
		write_user_ex_file(USR_OUT_RAWIMU, user_output_msg);
		//process
		write_user_process_file(USR_OUT_RAWIMU, 0, user_output_msg);
	}

	void output_user_g1() {
		double horizontal_speed = sqrt(pak_g1.north_vel * pak_g1.north_vel + pak_g1.east_vel * pak_g1.east_vel);
		double track_over_ground = atan2(pak_g1.east_vel, pak_g1.north_vel)*R2D;
		//csv
		sprintf(user_output_msg, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%3d,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n",
			pak_g1.GPS_Week, (double)pak_g1.GPS_TimeOfWeek / 1000.0, pak_g1.position_type, pak_g1.latitude, pak_g1.longitude, pak_g1.height,
			pak_g1.latitude_standard_deviation, pak_g1.longitude_standard_deviation, pak_g1.height_standard_deviation,
			pak_g1.number_of_satellites, pak_g1.number_of_satellites_in_solution, pak_g1.hdop, pak_g1.diffage, pak_g1.north_vel, pak_g1.east_vel, pak_g1.up_vel,
			pak_g1.north_vel_standard_deviation, pak_g1.east_vel_standard_deviation, pak_g1.up_vel_standard_deviation);
		write_user_log_file(USR_OUT_BESTGNSS, user_output_msg);
		//txt
		sprintf(user_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			pak_g1.GPS_Week, (double)pak_g1.GPS_TimeOfWeek / 1000.0, pak_g1.latitude, pak_g1.longitude, pak_g1.height,
			pak_g1.latitude_standard_deviation, pak_g1.longitude_standard_deviation, pak_g1.height_standard_deviation,
			pak_g1.position_type, pak_g1.north_vel, pak_g1.east_vel, pak_g1.up_vel, track_over_ground);
		write_user_ex_file(USR_OUT_BESTGNSS, user_output_msg);
		//process $GPGNSS
		sprintf(user_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f\n",
			pak_g1.GPS_Week, (double)pak_g1.GPS_TimeOfWeek / 1000.0, pak_g1.latitude, pak_g1.longitude, pak_g1.height,
			pak_g1.latitude_standard_deviation, pak_g1.longitude_standard_deviation, pak_g1.height_standard_deviation,
			pak_g1.position_type, pak_g1.diffage);
		write_user_process_file(USR_OUT_BESTGNSS, 0, user_output_msg);
		//process $GPVEL
		sprintf(user_output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n",
			pak_g1.GPS_Week, (double)pak_g1.GPS_TimeOfWeek / 1000.0, horizontal_speed, track_over_ground, pak_g1.up_vel);
		write_user_process_file(USR_OUT_BESTGNSS, 1, user_output_msg);
		//kml
		append_gnss_kml();
	}

	void output_user_i1() {
		//csv
		sprintf(user_output_msg, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n",
			pak_i1.GPS_Week, (double)pak_i1.GPS_TimeOfWeek / 1000.0, pak_i1.ins_status, pak_i1.ins_position_type, pak_i1.latitude, pak_i1.longitude, pak_i1.height,
			pak_i1.north_velocity, pak_i1.east_velocity, pak_i1.up_velocity, pak_i1.roll, pak_i1.pitch, pak_i1.heading, pak_i1.latitude_std, pak_i1.longitude_std, pak_i1.height_std,
			pak_i1.north_velocity_std, pak_i1.east_velocity_std, pak_i1.up_velocity_std, pak_i1.roll_std, pak_i1.pitch_std, pak_i1.heading_std);
		write_user_log_file(USR_OUT_INSPVAX, user_output_msg);
		if (pak_i1.GPS_TimeOfWeek % 100 == 0) {
			//txt
			sprintf(user_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n",
				pak_i1.GPS_Week, (double)pak_i1.GPS_TimeOfWeek / 1000.0, pak_i1.latitude, pak_i1.longitude, pak_i1.height,
				pak_i1.north_velocity, pak_i1.east_velocity, pak_i1.up_velocity,
				pak_i1.roll, pak_i1.pitch, pak_i1.heading, pak_i1.ins_position_type, pak_i1.ins_status);
			write_user_ex_file(USR_OUT_INSPVAX, user_output_msg);
			//process
			sprintf(user_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d\n",
				pak_i1.GPS_Week, (double)pak_i1.GPS_TimeOfWeek / 1000.0, pak_i1.latitude, pak_i1.longitude, pak_i1.height,
				pak_i1.north_velocity, pak_i1.east_velocity, pak_i1.up_velocity,
				pak_i1.roll, pak_i1.pitch, pak_i1.heading, pak_i1.ins_position_type);
			write_user_process_file(USR_OUT_INSPVAX, 0, user_output_msg);
		}
		//kml
		append_ins_kml();
	}

	void output_user_o1() {
		sprintf(user_output_msg, "%d,%11.4f,%d,%10.4f,%d,%I64d\n",
			pak_o1.GPS_Week, (double)pak_o1.GPS_TimeOfWeek / 1000.0, pak_o1.mode, pak_o1.speed, pak_o1.fwd, pak_o1.wheel_tick);
		write_user_log_file(USR_OUT_ODO, user_output_msg);
		write_user_ex_file(USR_OUT_ODO, user_output_msg);
	}

	void parse_user_packet_payload(uint8_t* buff, uint32_t nbyte) {
		uint8_t payload_lenth = buff[2];
		char packet_type[4] = { 0 };
		uint8_t* payload = buff + 3;
		char log_str[1024] = { 0 };
		memcpy(packet_type, buff, 2);
		if (strcmp(packet_type, "s1") == 0) {
			user_raw.ntype = USR_OUT_RAWIMU;
			size_t packet_size = sizeof(user_s1_t);
			if (payload_lenth == packet_size) {
				memcpy(&pak_s1, payload, packet_size);
				if (output_user_file) {
					output_user_s1();
					write_user_bin_file(USR_OUT_RAWIMU, buff, nbyte);
				}
			}
		}
		else if (strcmp(packet_type, "g1") == 0) {
			user_raw.ntype = USR_OUT_BESTGNSS;
			size_t packet_size = sizeof(user_g1_t);
			if (payload_lenth == packet_size) {
				memcpy(&pak_g1, payload, packet_size);
				if (output_user_file) {
					output_user_g1();
				}
			}
		}
		else if (strcmp(packet_type, "i1") == 0) {
			user_raw.ntype = USR_OUT_INSPVAX;
			size_t packet_size = sizeof(user_i1_t);
			if (payload_lenth == packet_size) {
				memcpy(&pak_i1, payload, packet_size);
				printf("0x%p\n", f_ins);
				if (output_user_file) {
					output_user_i1();
				}
			}
		}
		else if (strcmp(packet_type, "o1") == 0) {
			user_raw.ntype = USR_OUT_ODO;
			size_t packet_size = sizeof(user_o1_t);
			if (payload_lenth == packet_size) {
				memcpy(&pak_o1, payload, packet_size);
				if (output_user_file) {
					output_user_o1();
				}
			}
		}
		else if (strcmp(packet_type, "y1") == 0) {
			user_raw.ntype = USR_OUT_SATELLITES;
			char* p = user_output_msg;
			size_t packet_size = sizeof(user_y1_t);
			if (payload_lenth % packet_size == 0) {
				uint32_t num = payload_lenth / (uint32_t)packet_size;
				uint32_t i = 0;
				for (i = 0; i < num; i++) {
					memcpy(&pak_y1, payload + i * packet_size, packet_size);
					if (output_user_file) {
						sprintf(p, "%d,%11.4f,%4d,%4d,%5d,%4d,%4d,%10.3f,%10.3f\n", pak_y1.GPS_Week, (double)pak_y1.GPS_TimeOfWeek / 1000.0,
							pak_y1.satelliteId, pak_y1.systemId, pak_y1.antennaId, pak_y1.l1cn0, pak_y1.l2cn0, pak_y1.azimuth, pak_y1.elevation);
						p = user_output_msg + strlen(user_output_msg);
					}
				}
				if (output_user_file) {
					write_user_log_file(USR_OUT_SATELLITES, user_output_msg);
				}
			}
		}
	}

	int parse_user_nmea(uint8_t data) {
		if (user_raw.nmea_flag == 0) {
			if (NEAM_HEAD == data) {
				user_raw.nmea_flag = 1;
				user_raw.nmeabyte = 0;
				user_raw.nmea[user_raw.nmeabyte++] = data;
			}
		}
		else if (user_raw.nmea_flag == 1) {
			user_raw.nmea[user_raw.nmeabyte++] = data;
			if (user_raw.nmeabyte == 6) {
				int i = 0;
				char NMEA[8] = { 0 };
				memcpy(NMEA, user_raw.nmea, 6);
				for (i = 0; i < MAX_NMEA_TYPES; i++) {
					if (strcmp(NMEA, nmea_type(i)) == 0) {
						user_raw.nmea_flag = 2;
						break;
					}
				}
				if (user_raw.nmea_flag != 2) {
					user_raw.nmea_flag = 0;
				}
			}
		}
		else if (user_raw.nmea_flag == 2) {
			user_raw.nmea[user_raw.nmeabyte++] = data;
			if (user_raw.nmea[user_raw.nmeabyte - 1] == 0x0A || user_raw.nmea[user_raw.nmeabyte - 2] == 0x0D) {
				user_raw.nmea[user_raw.nmeabyte - 2] = 0x0A;
				user_raw.nmea[user_raw.nmeabyte - 1] = 0;
				user_raw.nmea_flag = 0;
				if (output_user_file) {
					write_user_log_file(0, (char*)user_raw.nmea);
				}
				return 2;
			}
		}
		return 0;
	}

	extern int input_user_raw(uint8_t data) {
		int ret = 0;
		if (user_raw.flag == 0) {
			user_raw.header[user_raw.header_len++] = data;
			if (user_raw.header_len == 1) {
				if (user_raw.header[0] != USER_PREAMB) {
					user_raw.header_len = 0;
				}
			}
			if (user_raw.header_len == 2) {
				if (user_raw.header[1] != USER_PREAMB) {
					user_raw.header_len = 0;
				}
			}
			if (user_raw.header_len == 4) {
				int i = 0;
				for (i = 0; i < MAX_PACKET_TYPES; i++) {
					const char* packetType = userPacketsTypeList[i];
					if (packetType[0] == user_raw.header[2] && packetType[1] == user_raw.header[3]) {
						user_raw.flag = 1;
						user_raw.buff[user_raw.nbyte++] = packetType[0];
						user_raw.buff[user_raw.nbyte++] = packetType[1];
						break;
					}
				}
				user_raw.header_len = 0;
			}
			return parse_user_nmea(data);
		}
		else {
			user_raw.buff[user_raw.nbyte++] = data;
			if (user_raw.nbyte == user_raw.buff[2] + 5) { //5 = [type1,type2,len] + [crc1,crc2]
				uint16_t packet_crc = 256 * user_raw.buff[user_raw.nbyte - 2] + user_raw.buff[user_raw.nbyte - 1];
				if (packet_crc == calc_crc(user_raw.buff, user_raw.nbyte - 2)) {
					parse_user_packet_payload(user_raw.buff, user_raw.nbyte);
					ret = 1;
				}
				user_raw.flag = 0;
				user_raw.nbyte = 0;
			}
		}
		return ret;
	}
	uint8_t get_current_type()
	{
		return user_raw.ntype;
	}

	user_i1_t* get_ins_sol() {
		return &pak_i1;
	}
}
#include "NPOS122_decoder.h"
#include <string.h>
#include "common.h"

namespace NPOS122_Tool {
#define SYNC_BYTES_NUM	4
#define NPOS122_HEADER_LEN 12
	const uint8_t SYNC_BYTES[SYNC_BYTES_NUM] = { 0xAC,0x55,0x96,0x83 };

	NPOS122_decoder::NPOS122_decoder()
	{
		init();
	}

	NPOS122_decoder::~NPOS122_decoder()
	{
	}

	void NPOS122_decoder::init()
	{
		pack_num = 0;
		crc_right_num = 0;
		crc_error_num = 0;
		memset(&raw, 0, sizeof(raw));
		memset(&raw_nmea, 0, sizeof(raw_nmea));
		memset(&msg_header, 0, sizeof(msg_header));
		memset(&imu, 0, sizeof(imu));
		memset(&pva, 0, sizeof(pva));
		memset(base_file_name, 0, 256);
		Kml_Generator::Instance()->init();
		Kml_Generator::Instance()->set_status_type_define(emNpos122StatusType);
	}

	void NPOS122_decoder::set_base_file_name(char * file_name)
	{
		strcpy(base_file_name, file_name);
	}

	void NPOS122_decoder::close_all_files()
	{
		FilesMap::iterator it;
		for (it = output_file_map.begin(); it != output_file_map.end(); it++) {
			if (it->second) fclose(it->second); it->second = NULL;
		}
		output_file_map.clear();
	}

	void NPOS122_decoder::create_file(FILE *& file, const char * suffix, const char * title)
	{
		if (strlen(base_file_name) == 0) return;
		if (file == NULL) {
			char file_name[256] = { 0 };
			sprintf(file_name, "%s_%s", base_file_name, suffix);
			file = fopen(file_name, "wb");
			if (file && title) {
				fprintf(file, title);
			}
		}
	}

	FILE* NPOS122_decoder::get_file(std::string file_suffix, const char * title) {
		FilesMap::iterator it = output_file_map.find(file_suffix);
		if (it == output_file_map.end()) {
			FILE* file = NULL;
			create_file(file, file_suffix.c_str(), title);
			output_file_map[file_suffix] = file;
		}
		return output_file_map[file_suffix];
	}

	void NPOS122_decoder::append_ins_kml()
	{
		ins_kml.gps_week = msg_header.gps_week;
		ins_kml.gps_secs = (double)msg_header.gps_millisecs / 1000.0;
		ins_kml.ins_status = pva.ins_status;
		ins_kml.ins_position_type = pva.pos_type;
		ins_kml.latitude = pva.latitude;
		ins_kml.longitude = pva.longitude;
		ins_kml.height = pva.height;
		ins_kml.north_velocity = (float)pva.vel_x;
		ins_kml.east_velocity = (float)pva.vel_y;
		ins_kml.up_velocity = (float)pva.vel_z;
		ins_kml.roll = (float)pva.roll;
		ins_kml.pitch = (float)pva.pitch;
		ins_kml.heading = (float)pva.azimuth;
		Kml_Generator::Instance()->append_ins(ins_kml);
	}

	void NPOS122_decoder::output_pva() {
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s),ins_status(),position_type()"
			",latitude(deg),longitude(deg),height(m)"
			",velocity_x(m/s),velocity_y(m/s),velocity_z(m/s)"
			",roll(deg),pitch(deg),azimuth(deg)"
			",latitude_std(),longitude_std(),height_std()"
			",velocity_x_std(m/s),velocity_y_std(m/s),velocity_z_std(m/s)"
			",roll_std(deg),pitch_std(deg),azimuth_std(deg)"
			",sol_status(),last_update_time()\n";
		FILE* f_pva_csv = get_file("pva.csv", title.c_str());
		if (!f_pva_csv) return;
		fprintf(f_pva_csv, "%d,%11.4f,%3d,%3d,", msg_header.gps_week, (double)msg_header.gps_millisecs / 1000.0, pva.ins_status, pva.pos_type);
		fprintf(f_pva_csv, "%14.9f,%14.9f,%10.4f,", pva.latitude, pva.longitude, pva.height + pva.undulation);
		fprintf(f_pva_csv, "%10.4f,%10.4f,%10.4f,", pva.vel_x, pva.vel_y, pva.vel_z);
		fprintf(f_pva_csv, "%14.9f,%14.9f,%14.9f,", pva.roll, pva.pitch, pva.azimuth);
		fprintf(f_pva_csv, "%8.3f,%8.3f,%8.3f,", pva.latitude_std, pva.longitude_std, pva.height_std);
		fprintf(f_pva_csv, "%8.3f,%8.3f,%8.3f,", pva.vel_x_std, pva.vel_y_std, pva.vel_z_std);
		fprintf(f_pva_csv, "%8.3f,%8.3f,%8.3f,", pva.roll_std, pva.pitch_std, pva.azimuth_std);
		fprintf(f_pva_csv, "%3d,%3d\n", pva.sol_status, pva.last_update_time);
	}

	void NPOS122_decoder::output_ins_txt()
	{
		FILE* f_ins_txt = get_file("ins.txt", NULL);
		if (!f_ins_txt) return;
		fprintf(f_ins_txt, "%d,%11.4f,", msg_header.gps_week, (double)msg_header.gps_millisecs / 1000.0);
		fprintf(f_ins_txt, "%14.9f,%14.9f,%10.4f,", pva.latitude, pva.longitude, pva.height + pva.undulation);
		fprintf(f_ins_txt, "%10.4f,%10.4f,%10.4f,", pva.vel_x, pva.vel_y, pva.vel_z);
		fprintf(f_ins_txt, "%14.9f,%14.9f,%14.9f,", pva.roll, pva.pitch, pva.azimuth);
		fprintf(f_ins_txt, "%3d,%3d\n", get_normal_pos_type(pva.pos_type), pva.ins_status);
	}

	void NPOS122_decoder::output_imu() {
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2)"
			",x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)"
			",status()\n";
		FILE* f_imu_csv = get_file("imu.csv", title.c_str());
		if (!f_imu_csv) return;
		fprintf(f_imu_csv, "%d,%11.4f"
			",%14.10f,%14.10f,%14.10f"
			",%14.10f,%14.10f,%14.10f"
			",%d\n"
			, msg_header.gps_week,imu.gps_second
			, imu.accel_x, imu.accel_y, imu.accel_z
			, imu.gyro_x, imu.gyro_y, imu.gyro_z
			, imu.status);
	}

	void NPOS122_decoder::parse_msg_header() {
		uint8_t* header_payload = raw.buff + NPOS122_HEADER_LEN;
		memcpy(&msg_header, header_payload, sizeof(msg_header));
	}

	void NPOS122_decoder::parse_packet_payload()
	{
		uint8_t* payload = raw.buff + NPOS122_HEADER_LEN + sizeof(msg_header);
		switch (msg_header.msg_id)
		{
		case MSG_PVA:
		{
			size_t packet_size = sizeof(pva_t);
			if (msg_header.msg_len == packet_size) {
				memcpy(&pva, payload, msg_header.msg_len);
				output_pva();
				output_ins_txt();
				append_ins_kml();
			}
		}break;
		case MSG_IMU:
		{
			size_t packet_size = sizeof(imu_t);
			if (msg_header.msg_len == packet_size) {
				memcpy(&imu, payload, msg_header.msg_len);
				output_imu();
			}
		}break;
		default:
			break;
		}
	}

	int8_t NPOS122_decoder::parse_nmea(uint8_t data) {
		int8_t ret = 0;
		if (raw_nmea.flag == 0) {
			if (NEAM_HEAD == data) {
				raw_nmea.flag = 1;
				raw_nmea.nbyte = 0;
				raw_nmea.nmea[raw_nmea.nbyte++] = data;
			}
		}
		else if (raw_nmea.flag == 1) {
			raw_nmea.nmea[raw_nmea.nbyte++] = data;
			if (raw_nmea.nbyte == 6) {
				int i = 0;
				char NMEA[8] = { 0 };
				memcpy(NMEA, raw_nmea.nmea, NMEA_HEADER_LEN);
				for (i = 0; i < MAX_NMEA_TYPES; i++) {
					if (strcmp(NMEA, nmea_type(i)) == 0) {
						raw_nmea.flag = 2;
						break;
					}
				}
				if (raw_nmea.flag != 2) {
					raw_nmea.flag = 0;
				}
			}
		}
		else if (raw_nmea.flag == 2) {
			raw_nmea.nmea[raw_nmea.nbyte++] = data;
			if (raw_nmea.nmea[raw_nmea.nbyte - 1] == 0x0A || raw_nmea.nmea[raw_nmea.nbyte - 2] == 0x0D) {
				raw_nmea.nmea[raw_nmea.nbyte - 2] = 0x0A;
				raw_nmea.nmea[raw_nmea.nbyte - 1] = 0;
				raw_nmea.flag = 0;
				FILE* f_nmea = get_file("nmea.txt", NULL);
				if(f_nmea) fprintf(f_nmea, (char*)raw_nmea.nmea);
				ret = 2;
			}
		}
		return ret;
	}

	int8_t NPOS122_decoder::input_data(uint8_t data)
	{
		int8_t ret = 0;
		if (raw.flag == 0) {
			raw.header[raw.header_len++] = data;
			if (raw.header_len <= SYNC_BYTES_NUM) {
				if (memcmp(SYNC_BYTES, raw.header, raw.header_len) == 0) {
					if (raw.header_len == SYNC_BYTES_NUM) {
						raw.flag = 1;
						memcpy(raw.buff, raw.header, SYNC_BYTES_NUM);
						raw.nbyte = SYNC_BYTES_NUM;
						raw.header_len = 0;
					}					
				}
				else {
					raw.header_len = 0;
				}
			}
		}
		else {
			if (raw.nbyte > NPOS122_BUFFER_LEN) {
				return -1;
			}
			raw.buff[raw.nbyte++] = data;
			if (raw.nbyte == 12) {
				memcpy(&raw.msg_len, &raw.buff[8], sizeof(uint32_t));
				if (raw.msg_len <= 0 || raw.msg_len >= NPOS122_BUFFER_LEN) {
					raw.flag = 0;
					raw.nbyte = 0;
					raw.msg_len = 0;
				}
			}
			else if (raw.msg_len > 0 && raw.nbyte == raw.msg_len + NPOS122_HEADER_LEN + CRC32_LEN) {
				uint32_t cal_crc = cal_crc_32(raw.buff, raw.nbyte - CRC32_LEN);
				uint32_t read_crc = 0;
				memcpy(&read_crc, &raw.buff[raw.nbyte - CRC32_LEN], CRC32_LEN);
				parse_msg_header();
				if (read_crc == cal_crc) {
					crc_right_num++;
					parse_packet_payload();
					ret = 1;
				}
				else {
					crc_error_num++;
					FILE* f_log = get_file(".log", NULL);
					if(f_log)fprintf(f_log, "crc failed read msg id = %d, len = %d, crc = %08X, calc crc = %08X\n", msg_header.msg_id, raw.msg_len, read_crc, cal_crc);
				}
				raw.flag = 0;
				raw.nbyte = 0;
				raw.msg_len = 0;
			}
		}
		return ret;
	}

	void NPOS122_decoder::finish() {
		Kml_Generator::Instance()->open_files(base_file_name);
		Kml_Generator::Instance()->write_files();
		Kml_Generator::Instance()->close_files();
		close_all_files();
	}
	int NPOS122_decoder::get_normal_pos_type(int pos_type)
	{
		int normal_type = 0;
		if (pos_type == 16 || pos_type == 53) {
			normal_type = 1;
		}
		else if (pos_type == 17 || pos_type == 54) {
			normal_type = 2;
		}
		else if (pos_type == 50 || pos_type == 56) {
			normal_type = 4;
		}
		else if (pos_type == 34 || pos_type == 55) {
			normal_type = 5;
		}
		return normal_type;
	}
}


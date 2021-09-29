#include "ins401.h"
#include <string.h>
#include <math.h>
#include "common.h"
#include "rtklib_core.h" //R2D
#include "rtkcmn.h"

using namespace Ins401;

enum emPackageType{
	em_RAW_IMU = 0x0a01,
	em_GNSS_SOL = 0x0a02,
	em_INS_SOL = 0x0a03,
	em_RAW_ODO = 0x0a04,
	em_DIAGNOSTIC_MSG = 0x0a05,
	em_ROVER_RTCM = 0x0a06,
	em_MISALIGN = 0x0a07	
};

Ins401::Ins401_decoder::Ins401_decoder()
{
	f_log = NULL;
	f_nmea = NULL;
	f_process = NULL;
	f_imu_csv = NULL;
	f_imu_txt = NULL;
	f_imu_bin = NULL;
	f_gnss_csv = NULL;
	f_gnss_txt = NULL;
	f_ins_csv = NULL;
	f_ins_txt = NULL;
	f_odo_csv = NULL;
	f_odo_txt = NULL;
	f_dm_csv = NULL;
	f_rover_rtcm = NULL;
	packets_type_list.clear();
	packets_type_list.push_back(em_RAW_IMU);
	packets_type_list.push_back(em_GNSS_SOL);
	packets_type_list.push_back(em_INS_SOL);
	packets_type_list.push_back(em_RAW_ODO);
	packets_type_list.push_back(em_DIAGNOSTIC_MSG);
	packets_type_list.push_back(em_ROVER_RTCM);
	packets_type_list.push_back(em_MISALIGN);
	all_type_pack_num.clear();
	init();
}

Ins401::Ins401_decoder::~Ins401_decoder()
{
	close_all_files();
}

void Ins401::Ins401_decoder::init()
{
	pack_num = 0;
	crc_right_num = 0;
	crc_error_num = 0;
	show_format_time = 0;
	memset(&raw, 0, sizeof(raw));
	memset(&imu, 0, sizeof(imu));
	memset(&gnss, 0, sizeof(gnss));
	memset(&ins, 0, sizeof(ins));
	memset(&odo, 0, sizeof(odo));
	memset(&gnss_kml, 0, sizeof(gnss_kml));
	memset(&ins_kml, 0, sizeof(ins_kml));	
	memset(base_file_name, 0, 256);
	memset(output_msg, 0, 1024);
	all_type_pack_num[em_RAW_IMU] = 0;
	all_type_pack_num[em_GNSS_SOL] = 0;
	all_type_pack_num[em_INS_SOL] = 0;
	all_type_pack_num[em_RAW_ODO] = 0;
	all_type_pack_num[em_DIAGNOSTIC_MSG] = 0;
	all_type_pack_num[em_ROVER_RTCM] = 0;
	all_type_pack_num[em_MISALIGN] = 0;
	Kml_Generator::Instance()->init();
}

void Ins401::Ins401_decoder::close_all_files()
{
	if (f_log)fclose(f_log); f_log = NULL;
	if (f_nmea)fclose(f_nmea); f_nmea = NULL;
	if (f_process)fclose(f_process); f_process = NULL;
	if (f_imu_csv)fclose(f_imu_csv); f_imu_csv = NULL;
	if (f_imu_txt)fclose(f_imu_txt); f_imu_txt = NULL;
	if (f_imu_bin)fclose(f_imu_bin); f_imu_bin = NULL;
	if (f_gnss_csv)fclose(f_gnss_csv); f_gnss_csv = NULL;
	if (f_gnss_txt)fclose(f_gnss_txt); f_gnss_txt = NULL;
	if (f_ins_csv)fclose(f_ins_csv); f_ins_csv = NULL;
	if (f_ins_txt)fclose(f_ins_txt); f_ins_txt = NULL;
	if (f_odo_csv)fclose(f_odo_csv); f_odo_csv = NULL;	
	if (f_odo_txt)fclose(f_odo_txt); f_odo_txt = NULL;	
	if (f_dm_csv)fclose(f_dm_csv); f_dm_csv = NULL;
	if (f_rover_rtcm)fclose(f_rover_rtcm); f_rover_rtcm = NULL;
}

void Ins401::Ins401_decoder::create_file(FILE* &file,const char* suffix, const char* title) {
	if (strlen(base_file_name) == 0) return;
	if (file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s", base_file_name, suffix);
		file = fopen(file_name, "wb");
		if (file && title) fprintf(file, title);
	}
}

void Ins401::Ins401_decoder::append_gnss_kml()
{
	gnss_kml.gps_week = gnss.gps_week;
	gnss_kml.gps_secs = (double)gnss.gps_millisecs / 1000.0;
	gnss_kml.position_type = gnss.position_type;
	gnss_kml.latitude = gnss.latitude;
	gnss_kml.longitude = gnss.longitude;
	gnss_kml.height = gnss.height;
	gnss_kml.north_vel = gnss.north_vel;
	gnss_kml.east_vel = gnss.east_vel;
	gnss_kml.up_vel = gnss.up_vel;
	Kml_Generator::Instance()->append_gnss(gnss_kml);
}

void Ins401::Ins401_decoder::append_ins_kml()
{
	ins_kml.gps_week = ins.gps_week;
	ins_kml.gps_secs = (double)ins.gps_millisecs / 1000.0;
	ins_kml.ins_status = ins.ins_status;
	ins_kml.ins_position_type = ins.ins_position_type;
	ins_kml.latitude = ins.latitude;
	ins_kml.longitude = ins.longitude;
	ins_kml.height = ins.height;
	ins_kml.north_velocity = ins.north_velocity;
	ins_kml.east_velocity = ins.east_velocity;
	ins_kml.up_velocity = ins.up_velocity;
	ins_kml.roll = ins.roll;
	ins_kml.pitch = ins.pitch;
	ins_kml.heading = ins.heading;
	Kml_Generator::Instance()->append_ins(ins_kml);
}

void Ins401::Ins401_decoder::output_imu_raw()
{
	//csv
	if (show_format_time) {
		create_file(f_imu_csv, "imu.csv",
			"DateTime(),GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");
		gtime_t gpstime = gpst2time(imu.gps_week, (double)imu.gps_millisecs / 1000.0);
		gtime_t utctime = gpst2utc(gpstime);
		char* time = time_str(utctime, 2);
		fprintf(f_imu_csv, "%s,%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", time,imu.gps_week, (double)imu.gps_millisecs / 1000.0,
			imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z);
	}
	else {
		create_file(f_imu_csv, "imu.csv",
			"GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");

		fprintf(f_imu_csv, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu.gps_week, (double)imu.gps_millisecs / 1000.0,
			imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z);
	}
#ifndef NOT_OUTPUT_INNER_FILE
	//txt
	create_file(f_imu_txt, "imu.txt",NULL);
	fprintf(f_imu_txt, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu.gps_week, (double)imu.gps_millisecs / 1000.0,
		imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z);
	//process
	create_file(f_process, "process", NULL);
	fprintf(f_process, "$GPIMU,%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu.gps_week, (double)imu.gps_millisecs / 1000.0,
		imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z);
#endif
}

void Ins401::Ins401_decoder::output_gnss_sol()
{
	double track_over_ground = atan2(gnss.east_vel, gnss.north_vel)*R2D;
	double horizontal_speed = sqrt(gnss.north_vel * gnss.north_vel + gnss.east_vel * gnss.east_vel);
	//csv
	if (show_format_time) {		
		create_file(f_gnss_csv, "gnss.csv",
			"DateTime(),GPS_Week(),GPS_TimeOfWeek(s),position_type(),latitude(deg),longitude(deg),height(m),latitude_standard_deviation(m),longitude_standard_deviation(m),height_standard_deviation(m),number_of_satellites(),number_of_satellites_in_solution(),hdop(),diffage(s),north_vel(m/s),east_vel(m/s),up_vel(m/s),north_vel_standard_deviation(m/s),east_vel_standard_deviation(m/s),up_vel_standard_deviation(m/s)\n");
		gtime_t gpstime = gpst2time(gnss.gps_week, (double)gnss.gps_millisecs / 1000.0);
		gtime_t utctime = gpst2utc(gpstime);
		char* time = time_str(utctime, 2);
		fprintf(f_gnss_csv, "%s,%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%3d,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n",
			time,gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.position_type, gnss.latitude, gnss.longitude, gnss.height,
			gnss.latitude_std, gnss.longitude_std, gnss.height_std,
			gnss.numberOfSVs, gnss.numberOfSVs_in_solution, gnss.hdop, gnss.diffage, gnss.north_vel, gnss.east_vel, gnss.up_vel,
			gnss.north_vel_std, gnss.east_vel_std, gnss.up_vel_std);
	}
	else {
		create_file(f_gnss_csv, "gnss.csv",
			"GPS_Week(),GPS_TimeOfWeek(s),position_type(),latitude(deg),longitude(deg),height(m),latitude_standard_deviation(m),longitude_standard_deviation(m),height_standard_deviation(m),number_of_satellites(),number_of_satellites_in_solution(),hdop(),diffage(s),north_vel(m/s),east_vel(m/s),up_vel(m/s),north_vel_standard_deviation(m/s),east_vel_standard_deviation(m/s),up_vel_standard_deviation(m/s)\n");

		fprintf(f_gnss_csv, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%3d,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n",
			gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.position_type, gnss.latitude, gnss.longitude, gnss.height,
			gnss.latitude_std, gnss.longitude_std, gnss.height_std,
			gnss.numberOfSVs, gnss.numberOfSVs_in_solution, gnss.hdop, gnss.diffage, gnss.north_vel, gnss.east_vel, gnss.up_vel,
			gnss.north_vel_std, gnss.east_vel_std, gnss.up_vel_std);
	}
#ifndef NOT_OUTPUT_INNER_FILE
	//txt
	create_file(f_gnss_txt, "gnssposvel.txt", NULL);
	fprintf(f_gnss_txt, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
		gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.latitude, gnss.longitude, gnss.height,
		gnss.latitude_std, gnss.longitude_std, gnss.height_std,
		gnss.position_type, gnss.north_vel, gnss.east_vel, gnss.up_vel, track_over_ground);
	//process
	create_file(f_process, "process", NULL);
	//process $GPGNSS
	fprintf(f_process, "$GPGNSS,%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d\n",
		gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.latitude, gnss.longitude, gnss.height,
		gnss.latitude_std, gnss.longitude_std, gnss.height_std,
		gnss.position_type);
	//process $GPVEL
	fprintf(f_process, "$GPVEL,%d,%11.4f,%10.4f,%10.4f,%10.4f\n",
		gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, horizontal_speed, track_over_ground, gnss.up_vel);
#endif
	append_gnss_kml();
}

void Ins401::Ins401_decoder::output_ins_sol() {
	//csv
	if (show_format_time) {
		create_file(f_ins_csv, "ins.csv",
			"DateTime(),GPS_Week(),GPS_TimeOfWeek(s),ins_status(),ins_position_type(),latitude(deg),longitude(deg),height(m),north_velocity(m/s),east_velocity(m/s),up_velocity(m/s),longitudinal_velocity(m/s),lateral_velocity(m/s),roll(deg),pitch(deg),heading(deg),latitude_std(m),longitude_std(m),height_std(m),north_velocity_std(m/s),east_velocity_std(m/s),up_velocity_std(m/s),long_vel_std(m/s),lat_vel_std(m/s),roll_std(deg),pitch_std(deg),heading_std(deg)\n");
		gtime_t gpstime = gpst2time(ins.gps_week, (double)ins.gps_millisecs / 1000.0);
		gtime_t utctime = gpst2utc(gpstime);
		char* time = time_str(utctime, 2);
		fprintf(f_ins_csv, "%s,%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n",
			time,ins.gps_week, (double)ins.gps_millisecs / 1000.0, ins.ins_status, ins.ins_position_type, ins.latitude, ins.longitude, ins.height,
			ins.north_velocity, ins.east_velocity, ins.up_velocity, ins.longitudinal_velocity, ins.lateral_velocity,
			ins.roll, ins.pitch, ins.heading, ins.latitude_std, ins.longitude_std, ins.height_std,
			ins.north_velocity_std, ins.east_velocity_std, ins.up_velocity_std,
			ins.long_vel_std, ins.lat_vel_std, ins.roll_std, ins.pitch_std, ins.heading_std);
	}
	else {
		create_file(f_ins_csv, "ins.csv",
			"GPS_Week(),GPS_TimeOfWeek(s),ins_status(),ins_position_type(),latitude(deg),longitude(deg),height(m),north_velocity(m/s),east_velocity(m/s),up_velocity(m/s),longitudinal_velocity(m/s),lateral_velocity(m/s),roll(deg),pitch(deg),heading(deg),latitude_std(m),longitude_std(m),height_std(m),north_velocity_std(m/s),east_velocity_std(m/s),up_velocity_std(m/s),long_vel_std(m/s),lat_vel_std(m/s),roll_std(deg),pitch_std(deg),heading_std(deg)\n");
		fprintf(f_ins_csv, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n",
			ins.gps_week, (double)ins.gps_millisecs / 1000.0, ins.ins_status, ins.ins_position_type, ins.latitude, ins.longitude, ins.height,
			ins.north_velocity, ins.east_velocity, ins.up_velocity, ins.longitudinal_velocity, ins.lateral_velocity,
			ins.roll, ins.pitch, ins.heading, ins.latitude_std, ins.longitude_std, ins.height_std,
			ins.north_velocity_std, ins.east_velocity_std, ins.up_velocity_std,
			ins.long_vel_std, ins.lat_vel_std, ins.roll_std, ins.pitch_std, ins.heading_std);
	}

#ifndef NOT_OUTPUT_INNER_FILE
	if (ins.gps_millisecs % 100 < 10) {
		//txt
		create_file(f_ins_txt, "ins.txt", NULL);
		fprintf(f_ins_txt, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n",
			ins.gps_week, (double)ins.gps_millisecs / 1000.0, ins.latitude, ins.longitude, ins.height,
			ins.north_velocity, ins.east_velocity, ins.up_velocity,
			ins.roll, ins.pitch, ins.heading, ins.ins_position_type, ins.ins_status);
		//process
		create_file(f_process, "process", NULL);
		fprintf(f_process, "$GPINS,%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d\n",
			ins.gps_week, (double)ins.gps_millisecs / 1000.0, ins.latitude, ins.longitude, ins.height,
			ins.north_velocity, ins.east_velocity, ins.up_velocity,
			ins.roll, ins.pitch, ins.heading, ins.ins_position_type);		
	}
#endif
	append_ins_kml();
}


void Ins401::Ins401_decoder::output_misa_sol() {
	if (show_format_time) {
		create_file(f_misa_csv, "misa.csv", "DateTime(),GPS_Week(),GPS_TimeOfWeek(s),flag(),RVB1(),RVB2(),RVB3(),CVB1(),CVB2(),CVB3()\n");
		gtime_t gpstime = gpst2time(odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0);
		gtime_t utctime = gpst2utc(gpstime);
		char* time = time_str(utctime, 2);
		fprintf(f_misa_csv, "%s,%d,%d,%d,%f,%f,%f,%f,%f,%f\n", time,misa.gps_week, misa.gps_millisecs, misa.flag, misa.RVB[0],misa.RVB[1],misa.RVB[2],misa.CVB[0],misa.CVB[1],misa.CVB[2]);
	}
	else {
		create_file(f_misa_csv, "misa.csv", "DateTime(),GPS_Week(),GPS_TimeOfWeek(s),flag(),RVB1(),RVB2(),RVB3(),CVB1(),CVB2(),CVB3()\n");
		fprintf(f_misa_csv, "%d,%d,%d,%f,%f,%f,%f,%f,%f\n", misa.gps_week, misa.gps_millisecs, misa.flag, misa.RVB[0],misa.RVB[1],misa.RVB[2],misa.CVB[0],misa.CVB[1],misa.CVB[2]);
	}

#ifndef NOT_OUTPUT_INNER_FILE
	create_file(f_misa_txt, "misa.txt", NULL);
	fprintf(f_misa_txt, "%d,%d,%d,%f,%f,%f,%f,%f,%f\n", misa.gps_week, misa.gps_millisecs, misa.flag, misa.RVB[0],misa.RVB[1],misa.RVB[2],misa.CVB[0],misa.CVB[1],misa.CVB[2]);
	create_file(f_process, "process", NULL);
	fprintf(f_process, "$GPMISA,%d,%d,%d,%f,%f,%f,%f,%f,%f\n", misa.gps_week, misa.gps_millisecs, misa.flag, misa.RVB[0],misa.RVB[1],misa.RVB[2],misa.CVB[0],misa.CVB[1],misa.CVB[2]);
#endif
}


void Ins401::Ins401_decoder::output_odo_raw()
{
	if (show_format_time) {
		create_file(f_odo_csv, "odo.csv", "DateTime(),GPS_Week(),GPS_TimeOfWeek(s),mode(),speed(m/s),fwd(),wheel_tick()\n");
		gtime_t gpstime = gpst2time(odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0);
		gtime_t utctime = gpst2utc(gpstime);
		char* time = time_str(utctime, 2);
		fprintf(f_odo_csv, "%s,%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", time,odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);
	}
	else {
		create_file(f_odo_csv, "odo.csv", "GPS_Week(),GPS_TimeOfWeek(s),mode(),speed(m/s),fwd(),wheel_tick()\n");
		fprintf(f_odo_csv, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);
	}

#ifndef NOT_OUTPUT_INNER_FILE
	create_file(f_odo_txt, "odo.txt", NULL);
	fprintf(f_odo_txt, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);

	create_file(f_process, "process", NULL);
	fprintf(f_process, "$GPODO,%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);
#endif
}

void Ins401::Ins401_decoder::output_dm_raw() {
	if (show_format_time) {
		create_file(f_dm_csv, "dm.csv", "DateTime(),GPS_Week(),GPS_TimeOfWeek(s),Device Status(),IMU Temperature(),MCU Temperature()\n");
		gtime_t gpstime = gpst2time(dm.gps_week, (double)dm.gps_millisecs / 1000.0);
		gtime_t utctime = gpst2utc(gpstime);
		char* time = time_str(utctime, 2);
		fprintf(f_dm_csv, "%s,%d,%11.3f,%3d,%7.1f,%7.1f\n", time,dm.gps_week, (double)dm.gps_millisecs / 1000.0, dm.Device_status_bit_field, dm.IMU_Unit_temperature, dm.MCU_temperature);
	}
	else {
		create_file(f_dm_csv, "dm.csv", "GPS_Week(),GPS_TimeOfWeek(s),Device Status(),IMU Temperature(),MCU Temperature()\n");
		fprintf(f_dm_csv, "%d,%11.3f,%3d,%7.1f,%7.1f\n", dm.gps_week, (double)dm.gps_millisecs / 1000.0, dm.Device_status_bit_field, dm.IMU_Unit_temperature, dm.MCU_temperature);
	}
}

void Ins401::Ins401_decoder::output_rover_rtcm()
{
	create_file(f_rover_rtcm, "_rover.rtcm", NULL);
	fwrite(raw.buff + 6, 1, raw.length, f_rover_rtcm);
}

void Ins401::Ins401_decoder::parse_packet_payload()
{
	uint8_t* payload = raw.buff + 6;
	switch (raw.packet_type) {
	case em_RAW_IMU:
	{
		size_t packet_size = sizeof(raw_imu_t);
		if (raw.length == packet_size) {
			memcpy(&imu, payload, packet_size);
			output_imu_raw();
#ifndef NOT_OUTPUT_INNER_FILE
			save_imu_bin();
#endif
		}
	}break;
	case em_GNSS_SOL:
	{
		size_t packet_size = sizeof(gnss_sol_t);
		if (raw.length == packet_size) {
			memcpy(&gnss, payload, packet_size);
			output_gnss_sol();
		}
	}break;
	case em_INS_SOL:
	{
		size_t packet_size = sizeof(ins_sol_t);
		if (raw.length == packet_size) {
			memcpy(&ins, payload, packet_size);
			output_ins_sol();
		}
	}break;
	case em_RAW_ODO:
	{
		size_t packet_size = sizeof(odo_t);
		if (raw.length == packet_size) {
			memcpy(&odo, payload, packet_size);
			output_odo_raw();
		}
	}break;
	case em_DIAGNOSTIC_MSG:
	{
		size_t packet_size = sizeof(diagnostic_msg_t);
		if (raw.length == packet_size) {
			memcpy(&dm, payload, packet_size);
			output_dm_raw();
		}
	}break;
	case em_ROVER_RTCM:
	{
		output_rover_rtcm();
	}break;
	case em_MISALIGN:
	{
		size_t packet_size = sizeof(binary_misalign_t);
		if (raw.length == packet_size) {
			memcpy(&misa, payload, packet_size);
			output_misa_sol();
		}
	}
	break;
	default:
		break;
	};
}

void Ins401::Ins401_decoder::save_imu_bin()
{
	create_file(f_imu_bin, "imu.bin",NULL);
	uint8_t buffer[128] = { 0 };
	buffer[0] = 's';
	buffer[1] = '1';
	uint8_t len = sizeof(imu);
	buffer[2] = len;
	memcpy(buffer + 3, &imu, len);
	uint16_t packet_crc = calc_crc(buffer, 3 + len);
	buffer[3 + len] = (packet_crc >> 8) & 0xff;
	buffer[3 + len + 1] = packet_crc & 0xff;
	fwrite(buffer, 1, len + 5, f_imu_bin);
}

int8_t Ins401::Ins401_decoder::parse_nmea(uint8_t data)
{
	if (raw.nmea_flag == 0) {
		if (NEAM_HEAD == data) {
			raw.nmea_flag = 1;
			raw.nmeabyte = 0;
			raw.nmea[raw.nmeabyte++] = data;
		}
	}
	else if (raw.nmea_flag == 1) {
		raw.nmea[raw.nmeabyte++] = data;
		if (raw.nmeabyte == 6) {
			int i = 0;
			char NMEA[8] = { 0 };
			memcpy(NMEA, raw.nmea, 6);
			for (i = 0; i < MAX_NMEA_TYPES; i++) {
				if (strcmp(NMEA, nmea_type(i)) == 0) {
					raw.nmea_flag = 2;
					break;
				}
			}
			if (raw.nmea_flag != 2) {
				raw.nmea_flag = 0;
			}
		}
	}
	else if (raw.nmea_flag == 2) {
		raw.nmea[raw.nmeabyte++] = data;
		if (raw.nmea[raw.nmeabyte - 1] == 0x0A || raw.nmea[raw.nmeabyte - 2] == 0x0D) {
			raw.nmea[raw.nmeabyte - 2] = 0x0A;
			raw.nmea[raw.nmeabyte - 1] = 0;
			raw.nmea_flag = 0;
			create_file(f_nmea, "nmea.txt",NULL);
			fprintf(f_nmea, (char*)raw.nmea);
			return 2;
		}
	}
	return 0;
}

void Ins401::Ins401_decoder::set_base_file_name(char * file_name)
{
	strcpy(base_file_name, file_name);
}

void Ins401::Ins401_decoder::set_show_format_time(bool show)
{
	show_format_time = show;
}

int Ins401::Ins401_decoder::input_data(uint8_t data)
{
	int ret = 0;
	if (raw.flag == 0) {
		raw.header[raw.header_len++] = data;
		if (raw.header_len == 1) {
			if (raw.header[0] != USER_PREAMB) {
				raw.header_len = 0;
			}
		}
		if (raw.header_len == 2) {
			if (raw.header[1] != USER_PREAMB) {
				raw.header_len = 0;
			}
		}
		if (raw.header_len == 4) {
			memcpy(&raw.packet_type, &raw.header[2], sizeof(uint16_t));
			for (int i = 0; i < packets_type_list.size(); i++) {
				uint16_t type_code = packets_type_list[i];
				if (raw.packet_type == type_code) {
					raw.flag = 1;
					raw.buff[raw.nbyte++] = raw.header[2];
					raw.buff[raw.nbyte++] = raw.header[3];
					break;
				}
			}
			raw.header_len = 0;
		}
		return parse_nmea(data);
	}
	else {
		raw.buff[raw.nbyte++] = data;
		if (raw.nbyte == 6) {
			memcpy(&raw.length, &raw.buff[2],sizeof(uint32_t));
		}
		else if (raw.length > 0 && raw.nbyte == raw.length + 8) { //8 = [type1,type2,len(4)] + [crc1,crc2]
			uint16_t packet_crc = 256 * raw.buff[raw.nbyte - 2] + raw.buff[raw.nbyte - 1];
			uint16_t crc = calc_crc(raw.buff, raw.nbyte - 2);
			pack_num++;
			all_type_pack_num[raw.packet_type]++;
			if (packet_crc == crc) {
				crc_right_num++;
				parse_packet_payload();
				ret = 1;
			}
			else {
				crc_error_num++;
				create_file(f_log, ".log", NULL);
				fprintf(f_log,"crc failed read type = %04X, len = %d, crc = %d, calc crc = %d\n", raw.packet_type, raw.length ,packet_crc, crc);
			}
			raw.flag = 0;
			raw.nbyte = 0;
			raw.length = 0;
			raw.packet_type = 0;
		}
	}
	return ret;
}

void Ins401::Ins401_decoder::finish()
{	
	create_file(f_log, ".log", NULL);
	for (std::map<uint16_t, int>::iterator it = all_type_pack_num.begin(); it != all_type_pack_num.end(); it++) {
		fprintf(f_log, "pack_type = 0x%04x, pack_num = %d\n", (uint16_t)it->first, (int)it->second);
	}
	fprintf(f_log, "all_pack_num = %d\ncrc_right_num = %d\ncrc_error_num = %d\n", pack_num, crc_right_num, crc_error_num);
	Kml_Generator::Instance()->open_files(base_file_name);
	Kml_Generator::Instance()->write_files();
	Kml_Generator::Instance()->close_files();
	close_all_files();
}
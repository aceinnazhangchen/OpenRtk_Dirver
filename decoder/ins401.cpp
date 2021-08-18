#include "ins401.h"
#include <string.h>
#include "rtklib_core.h" //R2D
#include "rtcm.h"
#include "openrtk_user.h"

using namespace Ins401;

#define INS401_PREAMB 0x55
#ifndef NEAM_HEAD
#define NEAM_HEAD 0x24
#endif // !NEAM_HEAD

extern const char* gnss_postype[6];
extern const char* ins_status[6];
extern const char* ins_postype[6];

enum emPackageType{
	em_RAW_IMU = 0x0a01,
	em_GNSS_SOL = 0x0a02,
	em_INS_SOL = 0x0a03,
	em_RAW_ODO = 0x0a04,
	em_DIAGNOSTIC_MSG = 0x0a05,
};

Ins401::Ins401_decoder::Ins401_decoder()
{
	kml_description = 1;
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
	f_gnss_kml = NULL;
	f_ins_kml = NULL;
	packets_type_list.clear();
	packets_type_list.push_back(em_RAW_IMU);
	packets_type_list.push_back(em_GNSS_SOL);
	packets_type_list.push_back(em_INS_SOL);
	packets_type_list.push_back(em_RAW_ODO);
	packets_type_list.push_back(em_DIAGNOSTIC_MSG);
	nmea_type_list.clear();
	nmea_type_list.push_back("$GPGGA");
	nmea_type_list.push_back("$GPZDA");
	init();
}

Ins401::Ins401_decoder::~Ins401_decoder()
{
	close_all_files();
}

void Ins401::Ins401_decoder::init()
{
	gnss_sol_list.clear();
	ins_sol_list.clear();
	memset(&raw, 0, sizeof(raw));
	memset(&imu, 0, sizeof(imu));
	memset(&gnss, 0, sizeof(gnss));
	memset(&ins, 0, sizeof(ins));
	memset(&odo, 0, sizeof(odo));
	memset(base_file_name, 0, 256);
	memset(output_msg, 0, 1024);
}

uint16_t Ins401::Ins401_decoder::calculate_crc(uint8_t * buf, uint16_t length)
{
	uint16_t crc = 0x1D0F;
	int i, j;
	for (i = 0; i < length; i++) {
		crc = crc ^ (buf[i] << 8);
		for (j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			}
			else {
				crc = crc << 1;
			}
		}
	}
	crc = crc & 0xffff;
	return crc;
}

void Ins401::Ins401_decoder::close_all_files()
{
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
	if (f_gnss_kml)fclose(f_gnss_kml); f_gnss_kml = NULL;
	if (f_ins_kml)fclose(f_ins_kml); f_ins_kml = NULL;
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

void Ins401::Ins401_decoder::write_gnss_kml_line(gnss_sol_t* gnss_pak, int begin_end) {
	if (f_gnss_kml == NULL) {
		create_file(f_gnss_kml, "gnss.kml", NULL);
		print_kml_header(f_gnss_kml, 0);
	}
	if (f_gnss_kml) {
		if (begin_end == 1) {
			fprintf(f_gnss_kml, "<Placemark>\n");
			fprintf(f_gnss_kml, "<name>Rover Track</name>\n");
			fprintf(f_gnss_kml, "<Style>\n");
			fprintf(f_gnss_kml, "<LineStyle>\n");
			fprintf(f_gnss_kml, "<color>ffffffff</color>\n");
			fprintf(f_gnss_kml, "</LineStyle>\n");
			fprintf(f_gnss_kml, "</Style>\n");
			fprintf(f_gnss_kml, "<LineString>\n");
			fprintf(f_gnss_kml, "<coordinates>\n");
		}
		if (fabs(gnss_pak->latitude) > 0.001) {
			fprintf(f_gnss_kml, "%.9f,%.9f,%.9f\n", gnss_pak->longitude, gnss_pak->latitude, gnss_pak->height);
		}
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</coordinates>\n");
			fprintf(f_gnss_kml, "</LineString>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
	}
}

void Ins401::Ins401_decoder::write_gnss_kml_point(gnss_sol_t* gnss_pak,int begin_end) {
	if (f_gnss_kml == NULL) {
		create_file(f_gnss_kml, "gnss.kml", NULL);
		print_kml_header(f_gnss_kml, 0);
	}
	if (f_gnss_kml) {
		if (begin_end == 1) {
			fprintf(f_gnss_kml, "<Folder>\n");
			fprintf(f_gnss_kml, "<name>Rover Position</name>\n");
		}
		if (fabs(gnss_pak->latitude) > 0.001) {
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(gnss_pak->gps_week, (double)gnss_pak->gps_millisecs / 1000.0);
			gtime_t utctime = gpst2utc(gpstime);
			time2epoch(utctime, ep);
			double track_ground = atan2(gnss_pak->east_vel, gnss_pak->north_vel)*R2D;
			fprintf(f_gnss_kml, "<Placemark>\n");
			if (begin_end == 1) {
				fprintf(f_gnss_kml, "<name>Start</name>\n");
			}
			else if (begin_end == -1) {
				fprintf(f_gnss_kml, "<name>End</name>\n");
			}
			fprintf(f_gnss_kml, "<TimeStamp><when>%04d-%02d-%02dT%02d:%02d:%05.2fZ</when></TimeStamp>\n", (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2], (int32_t)ep[3], (int32_t)ep[4], ep[5]);
			//=== description start ===
			if (kml_description) {
				fprintf(f_gnss_kml, "<description><![CDATA[\n");
				fprintf(f_gnss_kml, "<TABLE border=\"1\" width=\"100 % \" Align=\"center\">\n");
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT>\n");
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>%d</TD><TD>%.3f</TD><TD>%2d:%2d:%5.3f</TD><TD>%4d/%2d/%2d</TD></TR>\n",
					gnss_pak->gps_week, (double)gnss_pak->gps_millisecs / 1000.0, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					gnss_pak->latitude, gnss_pak->longitude, gnss_pak->height);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					gnss_pak->north_vel, gnss_pak->east_vel, -gnss_pak->up_vel);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%d</TD><TD>%d</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					0, 0, track_ground);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%d</TD><TD>%s</TD></TR>\n",
					0, gnss_postype[gnss_pak->position_type]);
				fprintf(f_gnss_kml, "</TABLE>\n");
				fprintf(f_gnss_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_gnss_kml, "<styleUrl>#P%d</styleUrl>\n", gnss_pak->position_type);
			fprintf(f_gnss_kml, "<Style>\n");
			fprintf(f_gnss_kml, "<IconStyle>\n");
			fprintf(f_gnss_kml, "<heading>%f</heading>\n", track_ground);
			fprintf(f_gnss_kml, "</IconStyle>\n");
			fprintf(f_gnss_kml, "</Style>\n");
			fprintf(f_gnss_kml, "<Point>\n");
			fprintf(f_gnss_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", gnss_pak->longitude, gnss_pak->latitude, gnss_pak->height);
			fprintf(f_gnss_kml, "</Point>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</Folder>\n");
		}
	}
}

void Ins401::Ins401_decoder::write_gnss_kml()
{
	//gnss kml
	for (int i = 0; i < gnss_sol_list.size(); ++i) {
		if (i == 0) {
			this->write_gnss_kml_line(&gnss_sol_list[i], 1);
		}
		else if (i == gnss_sol_list.size() - 1) {
			this->write_gnss_kml_line(&gnss_sol_list[i], -1);
		}
		else {
			this->write_gnss_kml_line(&gnss_sol_list[i], 0);
		}
	}
	for (int i = 0; i < gnss_sol_list.size(); ++i) {
		if (i == 0) {
			this->write_gnss_kml_point(&gnss_sol_list[i], 1);
		}
		else if (i == gnss_sol_list.size() - 1) {
			this->write_gnss_kml_point(&gnss_sol_list[i], -1);
		}
		else {
			this->write_gnss_kml_point(&gnss_sol_list[i], 0);
		}
	}
	gnss_sol_list.clear();
	print_kml_end(f_gnss_kml);
}

void Ins401_decoder::write_ins_kml_line(ins_sol_t* ins_pak, int begin_end) {
	if (f_ins_kml == NULL) {
		create_file(f_ins_kml, "ins.kml", NULL);
		print_kml_header(f_ins_kml, 1);
	}
	if (f_ins_kml) {
		if (begin_end == 1) {
			fprintf(f_ins_kml, "<Placemark>\n");
			fprintf(f_ins_kml, "<name>Rover Track</name>\n");
			fprintf(f_ins_kml, "<Style>\n");
			fprintf(f_ins_kml, "<LineStyle>\n");
			fprintf(f_ins_kml, "<color>ffffffff</color>\n");
			fprintf(f_ins_kml, "</LineStyle>\n");
			fprintf(f_ins_kml, "</Style>\n");
			fprintf(f_ins_kml, "<LineString>\n");
			fprintf(f_ins_kml, "<coordinates>\n");
		}
		fprintf(f_ins_kml, "%.9f,%.9f,%.9f\n", ins_pak->longitude, ins_pak->latitude, ins_pak->height);
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</coordinates>\n");
			fprintf(f_ins_kml, "</LineString>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
	}
}

void Ins401::Ins401_decoder::write_ins_kml_point(ins_sol_t * ins_pak, int begin_end)
{
	if (f_ins_kml == NULL) {
		create_file(f_ins_kml, "ins.kml", NULL);
		print_kml_header(f_ins_kml, 1);
	}
	if (f_ins_kml) {
		if (begin_end == 1) {
			fprintf(f_ins_kml, "<Folder>\n");
			fprintf(f_ins_kml, "<name>Rover Position</name>\n");
		}
		if (fabs(ins_pak->latitude*ins_pak->longitude) > 0.00000001)
		{
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(ins_pak->gps_week, (double)ins_pak->gps_millisecs / 1000.0);
			gtime_t utctime = gpst2utc(gpstime);
			time2epoch(utctime, ep);
			fprintf(f_ins_kml, "<Placemark>\n");
			if (begin_end == 1) {
				fprintf(f_ins_kml, "<name>Start</name>\n");
			}
			else if (begin_end == -1) {
				fprintf(f_ins_kml, "<name>End</name>\n");
			}
			fprintf(f_ins_kml, "<TimeStamp><when>%04d-%02d-%02dT%02d:%02d:%05.2fZ</when></TimeStamp>\n", (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2], (int32_t)ep[3], (int32_t)ep[4], ep[5]);
			//=== description start ===
			if (kml_description) {
				fprintf(f_ins_kml, "<description><![CDATA[\n");
				fprintf(f_ins_kml, "<TABLE border=\"1\" width=\"100 % \" Align=\"center\">\n");
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT>\n");
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>%d</TD><TD>%.3f</TD><TD>%2d:%2d:%5.3f</TD><TD>%4d/%2d/%2d</TD></TR>\n",
					ins_pak->gps_week, (double)ins_pak->gps_millisecs / 1000.0, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					ins_pak->latitude, ins_pak->longitude, ins_pak->height);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					ins_pak->north_velocity, ins_pak->east_velocity, -ins_pak->up_velocity);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					ins_pak->roll, ins_pak->pitch, ins_pak->heading);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%s</TD><TD>%s</TD></TR>\n",
					ins_status[ins_pak->ins_status], ins_postype[ins_pak->ins_position_type]);
				fprintf(f_ins_kml, "</TABLE>\n");
				fprintf(f_ins_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_ins_kml, "<styleUrl>#P%d</styleUrl>\n", ins_pak->ins_position_type);
			fprintf(f_ins_kml, "<Style>\n");
			fprintf(f_ins_kml, "<IconStyle>\n");
			fprintf(f_ins_kml, "<heading>%f</heading>\n", ins_pak->heading);
			fprintf(f_ins_kml, "</IconStyle>\n");
			fprintf(f_ins_kml, "</Style>\n");
			fprintf(f_ins_kml, "<Point>\n");
			fprintf(f_ins_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", ins_pak->longitude, ins_pak->latitude, ins_pak->height);
			fprintf(f_ins_kml, "</Point>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</Folder>\n");
		}
	}
}

void Ins401::Ins401_decoder::write_ins_kml()
{
	//gnss kml
	for (int i = 0; i < ins_sol_list.size(); ++i) {
		if (i == 0) {
			this->write_ins_kml_line(&ins_sol_list[i], 1);
		}
		else if (i == ins_sol_list.size() - 1) {
			this->write_ins_kml_line(&ins_sol_list[i], -1);
		}
		else {
			this->write_ins_kml_line(&ins_sol_list[i], 0);
		}
	}
	for (int i = 0; i < ins_sol_list.size(); ++i) {
		if (i == 0) {
			this->write_ins_kml_point(&ins_sol_list[i], 1);
		}
		else if (i == ins_sol_list.size() - 1) {
			this->write_ins_kml_point(&ins_sol_list[i], -1);
		}
		else {
			this->write_ins_kml_point(&ins_sol_list[i], 0);
		}
	}
	ins_sol_list.clear();
	print_kml_end(f_ins_kml);
}

void Ins401::Ins401_decoder::output_imu_raw()
{
	//csv
	create_file(f_imu_csv,"imu.csv",
		"GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");
	
	fprintf(f_imu_csv, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu.gps_week, (double)imu.gps_millisecs / 1000.0,
		imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z);
	//txt
	create_file(f_imu_txt, "imu.txt",NULL);
	fprintf(f_imu_txt, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu.gps_week, (double)imu.gps_millisecs / 1000.0,
		imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z);
	//process
	create_file(f_process, "process", NULL);
	fprintf(f_process, "$GPIMU,%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu.gps_week, (double)imu.gps_millisecs / 1000.0,
		imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y, imu.gyro_z);
}

void Ins401::Ins401_decoder::output_gnss_sol()
{
	double track_over_ground = atan2(gnss.east_vel, gnss.north_vel)*R2D;
	double horizontal_speed = sqrt(gnss.north_vel * gnss.north_vel + gnss.east_vel * gnss.east_vel);
	//csv
	create_file(f_gnss_csv, "gnss.csv",
		"GPS_Week(),GPS_TimeOfWeek(s),position_type(),latitude(deg),longitude(deg),height(m),latitude_standard_deviation(m),longitude_standard_deviation(m),height_standard_deviation(m),number_of_satellites(),number_of_satellites_in_solution(),hdop(),diffage(s),north_vel(m/s),east_vel(m/s),up_vel(m/s),north_vel_standard_deviation(m/s),east_vel_standard_deviation(m/s),up_vel_standard_deviation(m/s)\n");
	
	fprintf(f_gnss_csv, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%3d,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n",
		gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.position_type, gnss.latitude, gnss.longitude, gnss.height,
		gnss.latitude_std, gnss.longitude_std, gnss.height_std,
		gnss.numberOfSVs, gnss.numberOfSVs_in_solution, gnss.hdop, gnss.diffage, gnss.north_vel, gnss.east_vel, gnss.up_vel,
		gnss.north_vel_std, gnss.east_vel_std, gnss.up_vel_std);
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
	
	gnss_sol_list.push_back(gnss);
}

void Ins401::Ins401_decoder::output_ins_sol() {
	//csv
	create_file(f_ins_csv, "ins.csv",
		"GPS_Week(),GPS_TimeOfWeek(s),ins_status(),ins_position_type(),latitude(deg),longitude(deg),height(m),north_velocity(m/s),east_velocity(m/s),up_velocity(m/s),longitudinal_velocity(m/s),lateral_velocity(m/s),roll(deg),pitch(deg),heading(deg),latitude_std(m),longitude_std(m),height_std(m),north_velocity_std(m/s),east_velocity_std(m/s),up_velocity_std(m/s),long_vel_std(m/s),lat_vel_std(m/s),roll_std(deg),pitch_std(deg),heading_std(deg)\n");

	fprintf(f_ins_csv, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n",
		ins.gps_week, (double)ins.gps_millisecs / 1000.0, ins.ins_status, ins.ins_position_type, ins.latitude, ins.longitude, ins.height,
		ins.north_velocity, ins.east_velocity, ins.up_velocity, ins.longitudinal_velocity, ins.lateral_velocity,
		ins.roll, ins.pitch, ins.heading, ins.latitude_std, ins.longitude_std, ins.height_std,
		ins.north_velocity_std, ins.east_velocity_std, ins.up_velocity_std, 
		ins.long_vel_std, ins.lat_vel_std, ins.roll_std, ins.pitch_std, ins.heading_std);

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
	if (fabs(ins.latitude*ins.longitude) > 0.00000001 &&
		(((ins.gps_millisecs + 5) / 10) * 10) % 1000 == 0) {//1000 = 1hz;500 = 2hz;200 = 5hz;100 = 10hz;  x hz = 1000/x
		ins_sol_list.push_back(ins);
	}	
}

void Ins401::Ins401_decoder::output_odo_raw()
{
	create_file(f_odo_csv, "odo.csv", "GPS_Week(),GPS_TimeOfWeek(s),mode(),speed(m/s),fwd(),wheel_tick()\n");
	fprintf(f_odo_csv, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);

	create_file(f_odo_txt, "odo.txt", NULL);
	fprintf(f_odo_txt, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);

	create_file(f_process, "process", NULL);
	fprintf(f_process, "$GPODO,%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);
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
			save_imu_bin();
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
	}
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
			for (i = 0; i < nmea_type_list.size(); i++) {
				if (strcmp(NMEA, nmea_type_list[i].c_str()) == 0) {
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
		if (raw.nmea[raw.nmeabyte - 1] == 0x0A && raw.nmea[raw.nmeabyte - 2] == 0x0D) {
			raw.nmea[raw.nmeabyte - 2] = 0x0A;
			raw.nmea[raw.nmeabyte - 1] = 0;
			raw.nmea_flag = 0;
			create_file(f_nmea, "nmea",NULL);
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

int Ins401::Ins401_decoder::input_data(uint8_t data)
{
	int ret = 0;
	if (raw.flag == 0) {
		raw.header[raw.header_len++] = data;
		if (raw.header_len == 1) {
			if (raw.header[0] != INS401_PREAMB) {
				raw.header_len = 0;
			}
		}
		if (raw.header_len == 2) {
			if (raw.header[1] != INS401_PREAMB) {
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
			uint16_t calc_crc = calculate_crc(raw.buff, raw.nbyte - 2);
			if (packet_crc == calc_crc) {
				parse_packet_payload();
				ret = 1;
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
	write_gnss_kml();
	write_ins_kml();
	close_all_files();
}
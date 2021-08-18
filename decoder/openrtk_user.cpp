#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <string.h>
#include <math.h>
#include "openrtk_user.h"
#include "rtklib_core.h" //R2D
#include "rtcm.h"

const char* userPacketsTypeList[MAX_PACKET_TYPES] = { "s1", "g1", "i1", "o1", "y1" };
const char* userNMEAList[MAX_NMEA_TYPES] = { "$GPGGA", "$GPRMC", "$GPGSV", "$GLGSV", "$GAGSV", "$BDGSV", "$GPGSA", "$GLGSA", "$GAGSA", "$BDGSA", "$GPZDA", "$GPVTG", "$PASHR", "$GNINS" };
const char* gnss_postype[6] = {"NONE", "PSRSP", "PSRDIFF", "UNDEFINED", "RTKFIXED", "RTKFLOAT"};
const char* ins_status[6] = { "INS_INACTIVE", "INS_ALIGNING", "INS_HIGH_VARIANCE", "INS_SOLUTION_GOOD", "INS_SOLUTION_FREE", "INS_ALIGNMENT_COMPLETE" };
const char* ins_postype[6] = { "INS_NONE", "INS_PSRSP", "INS_PSRDIFF", "INS_PROPOGATED", "INS_RTKFIXED", "INS_RTKFLOAT" };

static usrRaw user_raw = { 0 };
static char user_output_msg[1024] = { 0 };
static int output_user_file = 0;
static user_s1_t pak_s1 = { 0 };
static user_g1_t pak_g1 = { 0 };
static user_i1_t pak_i1 = { 0 };
static user_o1_t pak_o1 = { 0 };
static user_y1_t pak_y1 = { 0 };
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
static FILE* f_gnss_kml = NULL;
static FILE* f_ins_kml = NULL;
static int save_bin = 0;
static FILE* fs1_b = NULL;
int kml_description = 1;
static char base_user_file_name[256] = { 0 };

extern void set_save_bin(int save) {
	save_bin = save;
}
extern void set_output_user_file(int output) {
	output_user_file = output;
}
extern void set_base_user_file_name(char* file_name)
{
	strcpy(base_user_file_name, file_name);
	memset(&user_raw, 0, sizeof(usrRaw));
}

extern uint16_t calc_crc(uint8_t* buff, uint32_t nbyte) {
	uint16_t crc = 0x1D0F;
	int i, j;
	for (i = 0; i < nbyte; i++) {
		crc = crc ^ (buff[i] << 8);
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

extern void print_kml_header(FILE *kml_file,int ntype) {//ntype 0:gnss 1:ins
	if (kml_file) {
		const char* color_gnss[6] = {
			"ffffffff","ff0000ff","ffff00ff","50FF78F0","ff00ff00","ff00aaff"
		};//B-G-R °×É« SPP RTD UDR FIX FLOAT
		const char* color_ins[6] = {
			"ffffffff","50FF78F0","ffff00ff","ff0000ff","ff00ff00","ff00aaff" 
		};
		int i;
		fprintf(kml_file, HEADKML1);
		fprintf(kml_file, HEADKML2);
		fprintf(kml_file, "<Document>\n");
		for (i = 0; i < 6; i++)
		{
			fprintf(kml_file, "<Style id=\"P%d\">\n", i);
			fprintf(kml_file, "<IconStyle>\n");
			if (ntype == 0) {
				fprintf(kml_file, "<color> %s </color>\n", color_gnss[i]);
			}else {
				fprintf(kml_file, "<color> %s </color>\n", color_ins[i]);
			}			
			fprintf(kml_file, "<scale> %f </scale>\n", SIZP);
			fprintf(kml_file, "<Icon><href> %s </href></Icon>\n", MARKICON);
			fprintf(kml_file, "</IconStyle>\n");
			fprintf(kml_file, "</Style>\n");
		}
	}
}
extern void print_kml_end(FILE *kml_file) {
	if (kml_file) {
		fprintf(kml_file, "</Document>\n");
		fprintf(kml_file, "</kml>\n");
	}
}
extern void close_user_all_log_file() {
	memset(&user_raw, 0, sizeof(usrRaw));
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
	if (f_gnss_kml) {print_kml_end(f_gnss_kml); fclose(f_gnss_kml);} f_gnss_kml = NULL;
	if (f_ins_kml) {print_kml_end(f_ins_kml); fclose(f_ins_kml);} f_ins_kml = NULL;

	if (fs1_b)fclose(fs1_b); fs1_b = NULL;
}

int get_user_packet_type()
{
	return user_raw.ntype;
}

user_s1_t * get_user_packet_s1()
{
	return &pak_s1;
}

user_g1_t * get_user_packet_g1()
{
	return &pak_g1;
}

user_i1_t * get_user_packet_i1()
{
	return &pak_i1;
}

void write_user_process_file(int index,int type, char* log) {
	if (strlen(base_user_file_name) == 0) return;
	char file_name[256] = { 0 };
	if (f_process == NULL) {
		sprintf(file_name, "%s-process", base_user_file_name);
		f_process = fopen(file_name, "w");
	}
	switch (index)
	{
	case USR_OUT_RAWIMU:
	{
		if (f_process) fprintf(f_process, "$GPIMU,%s",log);
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

void write_gnss_kml_line(user_g1_t* pak_gnss,int begin_end) {
	if (f_gnss_kml == NULL) {
		if (strlen(base_user_file_name) == 0) return;
		char file_name[256] = { 0 };
		sprintf(file_name, "%s-gnss.kml", base_user_file_name);
		f_gnss_kml = fopen(file_name, "w");
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
		if (fabs(pak_gnss->latitude) > 0.001) {
			fprintf(f_gnss_kml, "%.9f,%.9f,%.9f\n", pak_gnss->longitude, pak_gnss->latitude, pak_gnss->height);
		}
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</coordinates>\n");
			fprintf(f_gnss_kml, "</LineString>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
	}
}

void write_gnss_kml_file(user_g1_t* pak_gnss, int begin_end) {
	if (f_gnss_kml == NULL) {
		if (strlen(base_user_file_name) == 0) return;
		char file_name[256] = { 0 };
		sprintf(file_name, "%s-gnss.kml", base_user_file_name);
		f_gnss_kml = fopen(file_name, "w");
		print_kml_header(f_gnss_kml,0);
	}
	if (f_gnss_kml) {
		if (begin_end == 1) {
			fprintf(f_gnss_kml, "<Folder>\n");
			fprintf(f_gnss_kml, "<name>Rover Position</name>\n");
		}
		if (fabs(pak_gnss->latitude) > 0.001) {
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(pak_gnss->GPS_Week, (double)pak_gnss->GPS_TimeOfWeek / 1000.0);
			gtime_t utctime = gpst2utc(gpstime);
			time2epoch(utctime, ep);
			double track_ground = atan2(pak_gnss->east_vel, pak_gnss->north_vel)*R2D;
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
					pak_gnss->GPS_Week, (double)pak_gnss->GPS_TimeOfWeek / 1000.0, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					pak_gnss->latitude, pak_gnss->longitude, pak_gnss->height);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					pak_gnss->north_vel, pak_gnss->east_vel, -pak_gnss->up_vel);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%d</TD><TD>%d</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					0, 0, track_ground);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%d</TD><TD>%s</TD></TR>\n",
					0, gnss_postype[pak_gnss->position_type]);
				fprintf(f_gnss_kml, "</TABLE>\n");
				fprintf(f_gnss_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_gnss_kml, "<styleUrl>#P%d</styleUrl>\n", pak_gnss->position_type);
			fprintf(f_gnss_kml, "<Style>\n");
			fprintf(f_gnss_kml, "<IconStyle>\n");
			fprintf(f_gnss_kml, "<heading>%f</heading>\n", track_ground);
			fprintf(f_gnss_kml, "</IconStyle>\n");
			fprintf(f_gnss_kml, "</Style>\n");
			fprintf(f_gnss_kml, "<Point>\n");
			fprintf(f_gnss_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", pak_gnss->longitude, pak_gnss->latitude, pak_gnss->height);
			fprintf(f_gnss_kml, "</Point>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</Folder>\n");
		}
	}
}

void write_ins_kml_line(user_i1_t* pak_ins, int begin_end) {
	if (f_ins_kml == NULL) {
		if (strlen(base_user_file_name) == 0) return;
		char file_name[256] = { 0 };
		sprintf(file_name, "%s-ins.kml", base_user_file_name);
		f_ins_kml = fopen(file_name, "w");
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
		do {
			if (fabs(pak_ins->latitude*pak_ins->longitude) < 0.00000001) break;
			if ((((pak_ins->GPS_TimeOfWeek + 5) / 10) * 10) % 1000 != 0) break;//1000 = 1hz;500 = 2hz;200 = 5hz;100 = 10hz;  x hz = 1000/x
			fprintf(f_ins_kml, "%.9f,%.9f,%.9f\n", pak_ins->longitude, pak_ins->latitude, pak_ins->height);
		} while(0);
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</coordinates>\n");
			fprintf(f_ins_kml, "</LineString>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
	}
}

void write_ins_kml_file(user_i1_t* pak_ins, int begin_end) {
	if (strlen(base_user_file_name) == 0) return;
	char file_name[256] = { 0 };
	if (f_ins_kml == NULL) {
		sprintf(file_name, "%s-ins.kml", base_user_file_name);
		f_ins_kml = fopen(file_name, "w");
		print_kml_header(f_ins_kml,1);
	}
	if (f_ins_kml) {
		if (begin_end == 1) {
			fprintf(f_ins_kml, "<Folder>\n");
			fprintf(f_ins_kml, "<name>Rover Position</name>\n");
		}
		if (fabs(pak_ins->latitude*pak_ins->longitude) > 0.00000001 &&
			(((pak_ins->GPS_TimeOfWeek + 5) / 10) * 10) % 1000 == 0)//1000 = 1hz;500 = 2hz;200 = 5hz;100 = 10hz;  x hz = 1000/x
		{
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(pak_ins->GPS_Week, (double)pak_ins->GPS_TimeOfWeek / 1000.0);
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
					pak_ins->GPS_Week, (double)pak_ins->GPS_TimeOfWeek / 1000.0, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					pak_ins->latitude, pak_ins->longitude, pak_ins->height);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					pak_ins->north_velocity, pak_ins->east_velocity, -pak_ins->up_velocity);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					pak_ins->roll, pak_ins->pitch, pak_ins->heading);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%s</TD><TD>%s</TD></TR>\n",
					ins_status[pak_ins->ins_status], ins_postype[pak_ins->ins_position_type]);
				fprintf(f_ins_kml, "</TABLE>\n");
				fprintf(f_ins_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_ins_kml, "<styleUrl>#P%d</styleUrl>\n", pak_ins->ins_position_type);
			fprintf(f_ins_kml, "<Style>\n");
			fprintf(f_ins_kml, "<IconStyle>\n");
			fprintf(f_ins_kml, "<heading>%f</heading>\n", pak_ins->heading);
			fprintf(f_ins_kml, "</IconStyle>\n");
			fprintf(f_ins_kml, "</Style>\n");
			fprintf(f_ins_kml, "<Point>\n");
			fprintf(f_ins_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", pak_ins->longitude, pak_ins->latitude, pak_ins->height);
			fprintf(f_ins_kml, "</Point>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</Folder>\n");
		}
	}
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
		if (fs1_b) fwrite(buff,1, nbyte, fs1_b);
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
	sprintf(user_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d\n",
		pak_g1.GPS_Week, (double)pak_g1.GPS_TimeOfWeek / 1000.0, pak_g1.latitude, pak_g1.longitude, pak_g1.height,
		pak_g1.latitude_standard_deviation, pak_g1.longitude_standard_deviation, pak_g1.height_standard_deviation,
		pak_g1.position_type);
	write_user_process_file(USR_OUT_BESTGNSS, 0, user_output_msg);
	//process $GPVEL
	sprintf(user_output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n",
		pak_g1.GPS_Week, (double)pak_g1.GPS_TimeOfWeek / 1000.0, horizontal_speed, track_over_ground, pak_g1.up_vel);
	write_user_process_file(USR_OUT_BESTGNSS, 1, user_output_msg);
	//kml
	//write_gnss_kml_file(&pak_g1);
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
	//write_ins_kml_file(&pak_i1);
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
			int num = payload_lenth / packet_size;
			int i = 0;
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
				if (strcmp(NMEA, userNMEAList[i]) == 0) {
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
		if (user_raw.nmea[user_raw.nmeabyte - 1] == 0x0A && user_raw.nmea[user_raw.nmeabyte - 2] == 0x0D) {
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
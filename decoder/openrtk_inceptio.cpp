#include <stdio.h>
#include <stdint.h>
#include <memory.h>
#include <string.h>
#include <math.h>
#include "openrtk_inceptio.h"
#include "rtklib_core.h" //R2D
#include "rtkcmn.h"

#define MAX_INT 2147483648.0

#define VERSION_EARLY		0
#define VERSION_24_01_21	1

const char* inceptioPacketsTypeList[MAX_INCEPTIO_PACKET_TYPES] = { "s1", "gN","iN","d1","d2","sT","o1","fM","rt"};
const char* inceptioNMEAList[MAX_NMEA_TYPES] = { "$GPGGA", "$GPRMC", "$GPGSV", "$GLGSV", "$GAGSV", "$BDGSV", "$GPGSA", "$GLGSA", "$GAGSA", "$BDGSA", "$GPZDA", "$GPVTG", "$PASHR", "$GNINS" };
extern const char* gnss_postype[6];
extern const char* ins_status[6];
extern const char* ins_postype[6];

static int data_version = 0;
static usrRaw inceptio_raw = { 0 };
static char inceptio_output_msg[1024] = { 0 };
static inceptio_s1_t inceptio_pak_s1 = { 0 };
static inceptio_gN_early_t inceptio_pak_gN_early;
static inceptio_gN_t inceptio_pak_gN;
static inceptio_iN_t inceptio_pak_iN = { 0 };
static inceptio_d1_t inceptio_pak_d1 = { 0 };
static inceptio_d2_t inceptio_pak_d2 = { 0 };
static inceptio_sT_t inceptio_pak_sT = { 0 };
static inceptio_o1_t inceptio_pak_o1 = { 0 };

static int output_inceptio_file = 0;
static FILE* fnmea = NULL;
static FILE* fs1 = NULL;
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
static FILE* f_gnss_kml = NULL;
static FILE* f_ins_kml = NULL;
static FILE* fs1_b = NULL;
int inceptio_kml_description = 1;
static char base_inceptio_file_name[256] = { 0 };

extern void set_output_inceptio_file(int output) {
	output_inceptio_file = output;
}
extern void set_base_inceptio_file_name(char* file_name)
{
	strcpy(base_inceptio_file_name, file_name);
	init_inceptio_data();
}

extern void init_inceptio_data() {
	memset(&inceptio_raw, 0, sizeof(usrRaw));
	memset(&inceptio_pak_s1, 0, sizeof(inceptio_s1_t));
	memset(&inceptio_pak_gN_early, 0, sizeof(inceptio_gN_early_t));
	memset(&inceptio_pak_gN, 0, sizeof(inceptio_gN_t));
	memset(&inceptio_pak_iN, 0, sizeof(inceptio_iN_t));
	memset(&inceptio_pak_d1, 0, sizeof(inceptio_d1_t));
	memset(&inceptio_pak_d2, 0, sizeof(inceptio_d2_t));
	memset(&inceptio_pak_sT, 0, sizeof(inceptio_sT_t));
	memset(&inceptio_pak_o1, 0, sizeof(inceptio_o1_t));
}

extern void close_inceptio_all_log_file() {
	memset(&inceptio_raw, 0, sizeof(usrRaw));
	if (fnmea)fclose(fnmea); fnmea = NULL;
	if (fs1)fclose(fs1); fs1 = NULL;
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
	if (f_gnss_kml) { print_kml_end(f_gnss_kml); fclose(f_gnss_kml); } f_gnss_kml = NULL;
	if (f_ins_kml) { print_kml_end(f_ins_kml); fclose(f_ins_kml); } f_ins_kml = NULL;

	if (fs1_b)fclose(fs1_b); fs1_b = NULL;
}

extern int get_inceptio_packet_type() {
	return inceptio_raw.ntype;
}

extern inceptio_gN_t* get_inceptio_packet_gN() {
	return &inceptio_pak_gN;
}

extern inceptio_iN_t* get_inceptio_packet_iN() {
	return &inceptio_pak_iN;
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
			if (fd2) fprintf(fd2, "GPS_Week(),GPS_TimeofWeek(s),latitude_std(),longitude_std(),height_std(),north_vel_std(),east_vel_std(),up_vel_std(),hor_pos_pl(),ver_pos_pl(),hor_vel_pl(),ver_vel_pl(),pos_integrity_status(),vel_integrity_status()\n");
		}
		if (fd2) fprintf(fd2, log);
	}
	break;
	case INCEPTIO_OUT_STATUS:
	{
		if (fsT == NULL) {
			sprintf(file_name, "%s_sT.csv", base_inceptio_file_name);
			fsT = fopen(file_name, "w");
			if (fsT) fprintf(fsT, "GPS_Week(),GPS_TimeofWeek(s),year(),mouth(),day(),hour(),min(),sec(),imu_status(),imu_temperature(),mcu_temperature()\n");
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
	case INCEPTIO_OUT_SCALED1:
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
	break;
	}
}

void write_inceptio_gnss_kml_line(inceptio_gN_t* pak_gnss, int begin_end) {
	if (f_gnss_kml == NULL) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		sprintf(file_name, "%s-gnss.kml", base_inceptio_file_name);
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
		if (pak_gnss->positionMode != 0) {
			fprintf(f_gnss_kml, "%.9f,%.9f,%.9f\n", pak_gnss->longitude*180.0/MAX_INT, pak_gnss->latitude*180.0/MAX_INT, pak_gnss->height);
		}
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</coordinates>\n");
			fprintf(f_gnss_kml, "</LineString>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
	}
}
void write_inceptio_gnss_kml_file(inceptio_gN_t* pak_gnss, int begin_end) {
	if (f_gnss_kml == NULL) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		sprintf(file_name, "%s-gnss.kml", base_inceptio_file_name);
		f_gnss_kml = fopen(file_name, "w");
		print_kml_header(f_gnss_kml, 0);
	}
	if (f_gnss_kml) {
		if (begin_end == 1) {
			fprintf(f_gnss_kml, "<Folder>\n");
			fprintf(f_gnss_kml, "<name>Rover Position</name>\n");
		}
		if (pak_gnss->positionMode != 0) {
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(pak_gnss->GPS_Week, pak_gnss->GPS_TimeOfWeek);
			gtime_t utctime = gpst2utc(gpstime);
			time2epoch(utctime, ep);
			float north_vel = (float)pak_gnss->velocityNorth / 100.0;
			float east_vel = (float)pak_gnss->velocityEast / 100.0;
			float up_vel = (float)pak_gnss->velocityUp / 100.0;
			double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
			double track_over_ground = atan2(east_vel, north_vel) * R2D;
			fprintf(f_gnss_kml, "<Placemark>\n");
			if (begin_end == 1) {
				fprintf(f_gnss_kml, "<name>Start</name>\n");
			}
			else if (begin_end == -1) {
				fprintf(f_gnss_kml, "<name>End</name>\n");
			}
			fprintf(f_gnss_kml, "<TimeStamp><when>%04d-%02d-%02dT%02d:%02d:%05.2fZ</when></TimeStamp>\n", (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2], (int32_t)ep[3], (int32_t)ep[4], ep[5]);
			//=== description start ===
			if (inceptio_kml_description) {
				fprintf(f_gnss_kml, "<description><![CDATA[\n");
				fprintf(f_gnss_kml, "<TABLE border=\"1\" width=\"100 % \" Align=\"center\">\n");
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT>\n");
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>%d</TD><TD>%.3f</TD><TD>%2d:%2d:%5.3f</TD><TD>%4d/%2d/%2d</TD></TR>\n",
					pak_gnss->GPS_Week, pak_gnss->GPS_TimeOfWeek, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					pak_gnss->latitude * 180.0 / MAX_INT, pak_gnss->longitude * 180.0 / MAX_INT, pak_gnss->height);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					north_vel, east_vel, -up_vel);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%d</TD><TD>%d</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					0, 0, track_over_ground);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%d</TD><TD>%s</TD></TR>\n",
					0, gnss_postype[pak_gnss->positionMode]);
				fprintf(f_gnss_kml, "</TABLE>\n");
				fprintf(f_gnss_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_gnss_kml, "<styleUrl>#P%d</styleUrl>\n", pak_gnss->positionMode);
			fprintf(f_gnss_kml, "<Style>\n");
			fprintf(f_gnss_kml, "<IconStyle>\n");
			fprintf(f_gnss_kml, "<heading>%f</heading>\n", track_over_ground);
			fprintf(f_gnss_kml, "</IconStyle>\n");
			fprintf(f_gnss_kml, "</Style>\n");
			fprintf(f_gnss_kml, "<Point>\n");
			fprintf(f_gnss_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", pak_gnss->longitude * 180.0 / MAX_INT, pak_gnss->latitude * 180.0 / MAX_INT, pak_gnss->height);
			fprintf(f_gnss_kml, "</Point>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</Folder>\n");
		}
	}
}

void write_inceptio_ins_kml_line(inceptio_iN_t* pak_ins, int begin_end) {
	if (f_ins_kml == NULL) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		sprintf(file_name, "%s-ins.kml", base_inceptio_file_name);
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
			if (pak_ins->latitude == 0 || pak_ins->longitude == 0) break;
			uint32_t GPS_TimeOfWeek = (uint32_t)(pak_ins->GPS_TimeOfWeek * 1000);
			if ((((GPS_TimeOfWeek + 5) / 10) * 10) % 1000 != 0) break;//1000 = 1hz;500 = 2hz;200 = 5hz;100 = 10hz;  x hz = 1000/x
			fprintf(f_ins_kml, "%.9f,%.9f,%.9f\n", pak_ins->longitude * 180.0 / MAX_INT, pak_ins->latitude * 180.0 / MAX_INT, pak_ins->height);
		} while (0);
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</coordinates>\n");
			fprintf(f_ins_kml, "</LineString>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
	}
}

void write_inceptio_ins_kml_file(inceptio_iN_t* pak_ins, int begin_end) {
	if (f_ins_kml == NULL) {
		if (strlen(base_inceptio_file_name) == 0) return;
		char file_name[256] = { 0 };
		sprintf(file_name, "%s-ins.kml", base_inceptio_file_name);
		f_ins_kml = fopen(file_name, "w");
		print_kml_header(f_ins_kml, 1);
	}
	if (f_ins_kml) {
		if (begin_end == 1) {
			fprintf(f_ins_kml, "<Folder>\n");
			fprintf(f_ins_kml, "<name>Rover Position</name>\n");
		}
		uint32_t GPS_TimeOfWeek = (uint32_t)(pak_ins->GPS_TimeOfWeek * 1000);
		if (pak_ins->latitude && pak_ins->longitude &&
			(((GPS_TimeOfWeek + 5) / 10) * 10) % 1000 == 0)//1000 = 1hz;500 = 2hz;200 = 5hz;100 = 10hz;  x hz = 1000/x
		{
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(pak_ins->GPS_Week, pak_ins->GPS_TimeOfWeek);
			gtime_t utctime = gpst2utc(gpstime);
			time2epoch(utctime, ep);
			fprintf(f_ins_kml, "<Placemark>\n");
			fprintf(f_ins_kml, "<TimeStamp><when>%04d-%02d-%02dT%02d:%02d:%05.2fZ</when></TimeStamp>\n", (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2], (int32_t)ep[3], (int32_t)ep[4], ep[5]);
			//=== description start ===
			if (inceptio_kml_description) {
				fprintf(f_ins_kml, "<description><![CDATA[\n");
				fprintf(f_ins_kml, "<TABLE border=\"1\" width=\"100 % \" Align=\"center\">\n");
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT>\n");
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>%d</TD><TD>%.3f</TD><TD>%2d:%2d:%5.3f</TD><TD>%4d/%2d/%2d</TD></TR>\n",
					pak_ins->GPS_Week, pak_ins->GPS_TimeOfWeek, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					pak_ins->latitude * 180.0 / MAX_INT, pak_ins->longitude * 180.0 / MAX_INT, pak_ins->height);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					pak_ins->velocityNorth / 100.0, pak_ins->velocityEast / 100.0, -pak_ins->velocityUp / 100.0);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					pak_ins->roll / 100.0, pak_ins->pitch / 100.0, pak_ins->heading / 100.0);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%s</TD><TD>%s</TD></TR>\n",
					ins_status[pak_ins->insStatus], ins_postype[pak_ins->insPositionType]);
				fprintf(f_ins_kml, "</TABLE>\n");
				fprintf(f_ins_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_ins_kml, "<styleUrl>#P%d</styleUrl>\n", pak_ins->insPositionType);
			fprintf(f_ins_kml, "<Style>\n");
			fprintf(f_ins_kml, "<IconStyle>\n");
			fprintf(f_ins_kml, "<heading>%f</heading>\n", pak_ins->heading / 100.0);
			fprintf(f_ins_kml, "</IconStyle>\n");
			fprintf(f_ins_kml, "</Style>\n");
			fprintf(f_ins_kml, "<Point>\n");
			fprintf(f_ins_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", pak_ins->longitude * 180.0 / MAX_INT, pak_ins->latitude * 180.0 / MAX_INT, pak_ins->height);
			fprintf(f_ins_kml, "</Point>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</Folder>\n");
		}
	}
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
	//txt
	sprintf(inceptio_output_msg, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", inceptio_pak_s1.GPS_Week, inceptio_pak_s1.GPS_TimeOfWeek,
		inceptio_pak_s1.x_accel, inceptio_pak_s1.y_accel, inceptio_pak_s1.z_accel, inceptio_pak_s1.x_gyro, inceptio_pak_s1.y_gyro, inceptio_pak_s1.z_gyro);
	write_inceptio_ex_file(inceptio_raw.ntype, inceptio_output_msg);
	//process
	write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);
}

void output_inceptio_gN_early() {
	float north_vel = (float)inceptio_pak_gN_early.velocityNorth / 100.0;
	float east_vel = (float)inceptio_pak_gN_early.velocityEast / 100.0;
	float up_vel = (float)inceptio_pak_gN_early.velocityUp / 100.0;
	float latitude_std = (float)inceptio_pak_gN_early.latitude_std / 1000.0;
	float longitude_std = (float)inceptio_pak_gN_early.longitude_std / 1000.0;
	float height_std = (float)inceptio_pak_gN_early.height_std / 1000.0;
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
}

void output_inceptio_gN() {
	float north_vel = (float)inceptio_pak_gN.velocityNorth / 100.0;
	float east_vel = (float)inceptio_pak_gN.velocityEast / 100.0;
	float up_vel = (float)inceptio_pak_gN.velocityUp / 100.0;
	float latitude_std = (float)inceptio_pak_gN.latitude_std / 1000.0;
	float longitude_std = (float)inceptio_pak_gN.longitude_std / 1000.0;
	float height_std = (float)inceptio_pak_gN.height_std / 1000.0;
	float pos_hor_pl = (float)inceptio_pak_gN.pos_hor_pl / 1000.0;
	float pos_ver_pl = (float)inceptio_pak_gN.pos_ver_pl / 1000.0;
	float vel_hor_pl = (float)inceptio_pak_gN.vel_hor_pl / 1000.0;
	float vel_ver_pl = (float)inceptio_pak_gN.vel_ver_pl / 1000.0;
	double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
	double track_over_ground = atan2(east_vel, north_vel) * R2D;
	//csv
	sprintf(inceptio_output_msg, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%3d,%5.1f,%5.1f,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%3d\n", 
		inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek,
		inceptio_pak_gN.positionMode, (double)inceptio_pak_gN.latitude*180.0 / MAX_INT, (double)inceptio_pak_gN.longitude*180.0 / MAX_INT, inceptio_pak_gN.height, 
		inceptio_pak_gN.numberOfSVs,inceptio_pak_gN.hdop, inceptio_pak_gN.vdop, inceptio_pak_gN.tdop,(float)inceptio_pak_gN.diffage,
		north_vel, east_vel, up_vel, latitude_std, longitude_std, height_std,
		pos_hor_pl, pos_ver_pl, inceptio_pak_gN.pos_status, vel_hor_pl, vel_ver_pl, inceptio_pak_gN.vel_status);
	write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
	//txt
	sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
		inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek, inceptio_pak_gN.latitude*180.0 / MAX_INT, inceptio_pak_gN.longitude*180.0 / MAX_INT, inceptio_pak_gN.height,
		latitude_std, longitude_std, height_std,inceptio_pak_gN.positionMode, north_vel, east_vel, up_vel, track_over_ground);
	write_inceptio_ex_file(inceptio_raw.ntype, inceptio_output_msg);
	//process $GPGNSS
	sprintf(inceptio_output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d\n", inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek,
		(double)inceptio_pak_gN.latitude*180.0 / MAX_INT, (double)inceptio_pak_gN.longitude*180.0 / MAX_INT, inceptio_pak_gN.height, latitude_std, longitude_std, height_std, inceptio_pak_gN.positionMode);
	write_inceptio_process_file(inceptio_raw.ntype, 0, inceptio_output_msg);
	//process $GPVEL
	sprintf(inceptio_output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n", inceptio_pak_gN.GPS_Week, inceptio_pak_gN.GPS_TimeOfWeek, horizontal_speed, track_over_ground, up_vel);
	write_inceptio_process_file(inceptio_raw.ntype, 1, inceptio_output_msg);
	//kml
	//write_inceptio_gnss_kml_file(&inceptio_pak_gN);
}

void output_inceptio_iN() {
	//csv
	sprintf(inceptio_output_msg, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f\n", inceptio_pak_iN.GPS_Week, inceptio_pak_iN.GPS_TimeOfWeek,
		inceptio_pak_iN.insStatus, inceptio_pak_iN.insPositionType,
		(double)inceptio_pak_iN.latitude*180.0 / MAX_INT, (double)inceptio_pak_iN.longitude*180.0 / MAX_INT, inceptio_pak_iN.height,
		(float)inceptio_pak_iN.velocityNorth / 100.0, (float)inceptio_pak_iN.velocityEast / 100.0, (float)inceptio_pak_iN.velocityUp / 100.0,
		(float)inceptio_pak_iN.roll / 100.0, (float)inceptio_pak_iN.pitch / 100.0, (float)inceptio_pak_iN.heading / 100.0);
	write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
	if ((uint32_t)(inceptio_pak_iN.GPS_TimeOfWeek*1000) % 100 == 0) {
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
	//write_inceptio_ins_kml_file(&inceptio_pak_iN);
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
	sprintf(inceptio_output_msg, "%d,%11.4f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%3d,%3d\n", inceptio_pak_d2.GPS_Week, inceptio_pak_d2.GPS_TimeOfWeek,
		(float)inceptio_pak_d2.latitude_std / 100.0, (float)inceptio_pak_d2.longitude_std / 100.0, (float)inceptio_pak_d2.height_std / 100.0,
		(float)inceptio_pak_d2.north_vel_std / 100.0, (float)inceptio_pak_d2.east_vel_std / 100.0, (float)inceptio_pak_d2.up_vel_std / 100.0,
		inceptio_pak_d2.hor_pos_pl, inceptio_pak_d2.ver_pos_pl, inceptio_pak_d2.hor_vel_pl, inceptio_pak_d2.ver_vel_pl, inceptio_pak_d2.pos_integrity_status, inceptio_pak_d2.vel_integrity_status);
	write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
}

void output_inceptio_sT() {
	//csv
	sprintf(inceptio_output_msg, "%d,%11.4f,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%8.3f,%8.3f\n", inceptio_pak_sT.GPS_Week, inceptio_pak_sT.GPS_TimeOfWeek,
		inceptio_pak_sT.year, inceptio_pak_sT.mouth, inceptio_pak_sT.day, inceptio_pak_sT.hour, inceptio_pak_sT.min, inceptio_pak_sT.sec,
		inceptio_pak_sT.imu_status, inceptio_pak_sT.imu_temperature, inceptio_pak_sT.mcu_temperature);
	write_inceptio_log_file(inceptio_raw.ntype, inceptio_output_msg);
}

void output_inceptio_o1() {
	//csv
	sprintf(inceptio_output_msg, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", inceptio_pak_o1.GPS_Week, (double)inceptio_pak_o1.GPS_TimeOfWeek/1000.0, inceptio_pak_o1.mode,
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
	else if (strcmp(packet_type, "gN") == 0) {
		inceptio_raw.ntype = INCEPTIO_OUT_GNSS;
		if (payload_lenth == sizeof(inceptio_gN_early_t)) {
			data_version = VERSION_EARLY;
			memcpy(&inceptio_pak_gN_early, payload, payload_lenth);
			output_inceptio_gN_early();
		}
		if (payload_lenth == sizeof(inceptio_gN_24_01_21_t) ||
			payload_lenth == sizeof(inceptio_gN_t)) {
			data_version = VERSION_24_01_21;
			memcpy(&inceptio_pak_gN, payload, payload_lenth);
			output_inceptio_gN();
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
				if (strcmp(NMEA, inceptioNMEAList[i]) == 0) {
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
		inceptio_raw.nmea[inceptio_raw.nmeabyte++] = data;
		if (inceptio_raw.nmea[inceptio_raw.nmeabyte - 1] == 0x0A && inceptio_raw.nmea[inceptio_raw.nmeabyte - 2] == 0x0D) {
			inceptio_raw.nmea[inceptio_raw.nmeabyte - 2] = 0x0A;
			inceptio_raw.nmea[inceptio_raw.nmeabyte - 1] = 0;
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
			if (packet_crc == calc_crc(inceptio_raw.buff, inceptio_raw.nbyte - 2)) {
				parse_inceptio_packet_payload(inceptio_raw.buff, inceptio_raw.nbyte);
				ret = 1;
			}
			inceptio_raw.flag = 0;
			inceptio_raw.nbyte = 0;
		}
	}
	return ret;
}
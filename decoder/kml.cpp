#include "kml.h"
#include <math.h>
#include "rtklib_core.h" //R2D
#include "rtkcmn.h"

const char* gnss_postype[6] = { "NONE", "PSRSP", "PSRDIFF", "UNDEFINED", "RTKFIXED", "RTKFLOAT" };
const char* ins_postype[6] = { "INS_NONE", "INS_PSRSP", "INS_PSRDIFF", "INS_PROPOGATED", "INS_RTKFIXED", "INS_RTKFLOAT" };
const char* ins_status[6] = { "INS_INACTIVE", "INS_ALIGNING", "INS_HIGH_VARIANCE", "INS_SOLUTION_GOOD", "INS_SOLUTION_FREE", "INS_ALIGNMENT_COMPLETE" };

Kml_Generator * Kml_Generator::m_instance = NULL;

Kml_Generator::Kml_Generator()
{
	ins_kml_frequency = 1000;
	kml_description = 1;
	f_gnss_kml = NULL;
	f_ins_kml = NULL;
	init();
}

Kml_Generator::~Kml_Generator()
{
	init();
	close_files();
}

Kml_Generator * Kml_Generator::Instance()
{
	if (m_instance == NULL)
	{
		createInstance();
	}
	return m_instance;
}

void Kml_Generator::createInstance()
{
	m_instance = new Kml_Generator();
}

void Kml_Generator::init()
{
	gnss_sol_list.clear();
	ins_sol_list.clear();
	ins_kml_frequency = 1000;
}

void Kml_Generator::set_kml_frequency(int frequency)
{
	ins_kml_frequency = frequency;
}

void Kml_Generator::open_files(char * file_base_name)
{
	char file_name[256] = { 0 };
	sprintf(file_name, "%s_%s", file_base_name, "gnss.kml");
	f_gnss_kml = fopen(file_name, "wb");
	sprintf(file_name, "%s_%s", file_base_name, "ins.kml");
	f_ins_kml = fopen(file_name, "wb");
}

void Kml_Generator::write_files()
{
	write_gnss_kml();
	write_ins_kml();
}

void Kml_Generator::close_files()
{
	if (f_gnss_kml)fclose(f_gnss_kml); f_gnss_kml = NULL;
	if (f_ins_kml)fclose(f_ins_kml); f_ins_kml = NULL;
}

void Kml_Generator::append_gnss(kml_gnss_t & gnss)
{
	if (fabs(gnss.latitude) > 0.001 && gnss.position_type != 0) {
		gnss_sol_list.push_back(gnss);
	}
}

void Kml_Generator::append_ins(kml_ins_t & ins)
{
	uint32_t gps_millisecs = (uint32_t)(ins.gps_secs * 1000);
	if (fabs(ins.latitude*ins.longitude) > 0.00000001 &&
		(((gps_millisecs + 5) / 10) * 10) % ins_kml_frequency == 0) {//1000 = 1hz;500 = 2hz;200 = 5hz;100 = 10hz;10 = 100hz x hz = 1000/x
		ins_sol_list.push_back(ins);
	}
}

void Kml_Generator::write_header(FILE * kml_file, int ntype)
{
	if (kml_file) {
		//A-B-G-R  SPP RTD UDR FIX FLOAT
		const char* color_gnss[6] = {
			"ffffffff","ff0000ff","ffff00ff","00000000","ff00ff00","ff00aaff"
		};
		const char* color_ins[6] = {
			"ffffffff","ff0000ff","ffff00ff","ffff901e","ff00ff00","ff00aaff"
			/*write,    read,	   purple,    blue,      green,     orange*/
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
			}
			else {
				fprintf(kml_file, "<color> %s </color>\n", color_ins[i]);
			}
			fprintf(kml_file, "<scale> %f </scale>\n", SIZP);
			fprintf(kml_file, "<Icon><href> %s </href></Icon>\n", MARKICON);
			fprintf(kml_file, "</IconStyle>\n");
			fprintf(kml_file, "</Style>\n");
		}
	}
}

void Kml_Generator::write_end(FILE * kml_file)
{
	if (kml_file) {
		fprintf(kml_file, "</Document>\n");
		fprintf(kml_file, "</kml>\n");
	}
}

void Kml_Generator::write_gnss_line(kml_gnss_t * gnss, int begin_end)
{
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
		fprintf(f_gnss_kml, "%.9f,%.9f,%.9f\n", gnss->longitude, gnss->latitude, gnss->height);
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</coordinates>\n");
			fprintf(f_gnss_kml, "</LineString>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
	}
}

void Kml_Generator::write_gnss_point(kml_gnss_t * gnss, int begin_end)
{
	if (f_gnss_kml) {
		if (begin_end == 1) {
			fprintf(f_gnss_kml, "<Folder>\n");
			fprintf(f_gnss_kml, "<name>Rover Position</name>\n");
		}
		{
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(gnss->gps_week, gnss->gps_secs);
			gtime_t utctime = gpst2utc(gpstime);
			time2epoch(utctime, ep);
			double track_ground = atan2(gnss->east_vel, gnss->north_vel)*R2D;
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
					gnss->gps_week, (double)gnss->gps_secs, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					gnss->latitude, gnss->longitude, gnss->height);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					gnss->north_vel, gnss->east_vel, -gnss->up_vel);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%d</TD><TD>%d</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					0, 0, track_ground);
				fprintf(f_gnss_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%d</TD><TD>%s</TD></TR>\n",
					0, gnss_postype[gnss->position_type]);
				fprintf(f_gnss_kml, "</TABLE>\n");
				fprintf(f_gnss_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_gnss_kml, "<styleUrl>#P%d</styleUrl>\n", gnss->position_type);
			fprintf(f_gnss_kml, "<Style>\n");
			fprintf(f_gnss_kml, "<IconStyle>\n");
			fprintf(f_gnss_kml, "<heading>%f</heading>\n", track_ground);
			fprintf(f_gnss_kml, "</IconStyle>\n");
			fprintf(f_gnss_kml, "</Style>\n");
			fprintf(f_gnss_kml, "<Point>\n");
			fprintf(f_gnss_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", gnss->longitude, gnss->latitude, gnss->height);
			fprintf(f_gnss_kml, "</Point>\n");
			fprintf(f_gnss_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_gnss_kml, "</Folder>\n");
		}
	}
}

void Kml_Generator::write_ins_line(kml_ins_t * ins, int begin_end)
{
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
		fprintf(f_ins_kml, "%.9f,%.9f,%.9f\n", ins->longitude, ins->latitude, ins->height);
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</coordinates>\n");
			fprintf(f_ins_kml, "</LineString>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
	}
}

void Kml_Generator::write_ins_point(kml_ins_t * ins, int begin_end)
{
	if (f_ins_kml) {
		if (begin_end == 1) {
			fprintf(f_ins_kml, "<Folder>\n");
			fprintf(f_ins_kml, "<name>Rover Position</name>\n");
		}
		{
			double ep[6] = { 0 };
			gtime_t gpstime = gpst2time(ins->gps_week, ins->gps_secs);
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
					ins->gps_week, ins->gps_secs, (int32_t)ep[3], (int32_t)ep[4], ep[5], (int32_t)ep[0], (int32_t)ep[1], (int32_t)ep[2]);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%.9f</TD><TD>%.9f</TD><TD>%.4f</TD><TD>(DMS,m)</TD></TR>\n",
					ins->latitude, ins->longitude, ins->height);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(m/s)</TD></TR>\n",
					ins->north_velocity, ins->east_velocity, -ins->up_velocity);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(deg,approx)</TD></TR>\n",
					ins->roll, ins->pitch, ins->heading);
				fprintf(f_ins_kml, "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>%s</TD><TD>%s</TD></TR>\n",
					ins_status[ins->ins_status], ins_postype[ins->ins_position_type]);
				fprintf(f_ins_kml, "</TABLE>\n");
				fprintf(f_ins_kml, "]]></description>\n");
			}
			//=== description end ===
			fprintf(f_ins_kml, "<styleUrl>#P%d</styleUrl>\n", ins->ins_position_type);
			fprintf(f_ins_kml, "<Style>\n");
			fprintf(f_ins_kml, "<IconStyle>\n");
			fprintf(f_ins_kml, "<heading>%f</heading>\n", ins->heading);
			fprintf(f_ins_kml, "</IconStyle>\n");
			fprintf(f_ins_kml, "</Style>\n");
			fprintf(f_ins_kml, "<Point>\n");
			fprintf(f_ins_kml, "<coordinates>%13.9f,%12.9f,%5.3f</coordinates>\n", ins->longitude, ins->latitude, ins->height);
			fprintf(f_ins_kml, "</Point>\n");
			fprintf(f_ins_kml, "</Placemark>\n");
		}
		if (begin_end == -1) {
			fprintf(f_ins_kml, "</Folder>\n");
		}
	}
}

void Kml_Generator::write_gnss_kml()
{
	if (f_gnss_kml) {
		write_header(f_gnss_kml, 0);
		for (int i = 0; i < gnss_sol_list.size(); ++i) {
			if (i == 0) {
				write_gnss_line(&gnss_sol_list[i], 1);
			}
			else if (i == gnss_sol_list.size() - 1) {
				write_gnss_line(&gnss_sol_list[i], -1);
			}
			else {
				write_gnss_line(&gnss_sol_list[i], 0);
			}
		}
		for (int i = 0; i < gnss_sol_list.size(); ++i) {
			if (i == 0) {
				write_gnss_point(&gnss_sol_list[i], 1);
			}
			else if (i == gnss_sol_list.size() - 1) {
				write_gnss_point(&gnss_sol_list[i], -1);
			}
			else {
				write_gnss_point(&gnss_sol_list[i], 0);
			}
		}
		gnss_sol_list.clear();
		write_end(f_gnss_kml);
	}
}

void Kml_Generator::write_ins_kml()
{
	if (f_ins_kml) {
		write_header(f_ins_kml, 1);
		for (int i = 0; i < ins_sol_list.size(); ++i) {
			if (i == 0) {
				write_ins_line(&ins_sol_list[i], 1);
			}
			else if (i == ins_sol_list.size() - 1) {
				write_ins_line(&ins_sol_list[i], -1);
			}
			else {
				write_ins_line(&ins_sol_list[i], 0);
			}
		}
		for (int i = 0; i < ins_sol_list.size(); ++i) {
			if (i == 0) {
				write_ins_point(&ins_sol_list[i], 1);
			}
			else if (i == ins_sol_list.size() - 1) {
				write_ins_point(&ins_sol_list[i], -1);
			}
			else {
				write_ins_point(&ins_sol_list[i], 0);
			}
		}
		ins_sol_list.clear();
		write_end(f_ins_kml);
	}
}

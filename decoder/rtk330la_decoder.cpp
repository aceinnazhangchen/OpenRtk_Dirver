#include "rtk330la_decoder.h"
#include "common.h"
#include "rtklib_core.h"

namespace RTK330LA_Tool {

	Rtk330la_decoder::Rtk330la_decoder()
		: m_isOutputFile(true)
	{
		all_type_pack_num.clear();
		output_file_map.clear();
		init();
	}


	Rtk330la_decoder::~Rtk330la_decoder()
	{
		close_all_files();
	}


	void Rtk330la_decoder::close_all_files()
	{
		FilesMap::iterator it;
		for (it = output_file_map.begin(); it != output_file_map.end(); it++) {
			if (it->second) fclose(it->second); it->second = NULL;
		}
		output_file_map.clear();
	}

	void Rtk330la_decoder::create_file(FILE *& file, const char * suffix, const char * title, bool format_time = false)
	{
		if (strlen(base_file_name) == 0) return;
		if (file == NULL) {
			char file_name[256] = { 0 };
			sprintf(file_name, "%s_%s", base_file_name, suffix);
			file = fopen(file_name, "wb");
			if (file && title) {
				if (format_time)
					fprintf(file, "DateTime(),");
				fprintf(file, title);
			}
		}
	}

	FILE * Rtk330la_decoder::get_file(std::string suffix, std::string title = "", bool format_time = false)
	{
		FILE* f_ptr = NULL;
		FilesMap::iterator it = output_file_map.find(suffix);
		if (it == output_file_map.end()) {
			create_file(f_ptr, suffix.c_str(), title.c_str(), format_time);
			output_file_map[suffix] = f_ptr;
		}
		f_ptr = output_file_map[suffix];
		return f_ptr;
	}

	void Rtk330la_decoder::init()
	{
		m_isOutputFile = true;
		data_version = 0;
		pack_num = 0;
		crc_right_num = 0;
		crc_error_num = 0;
		show_format_time = false;
		memset(base_file_name, 0, 256);
		memset(output_msg, 0, MAX_OUTPUT_MSG_LEN);
		memset(&raw, 0, sizeof(raw));
		memset(&pak_s1, 0, sizeof(pak_s1));
		memset(&pak_s2, 0, sizeof(pak_s2));
		memset(&pak_gN_early, 0, sizeof(pak_gN_early));
		memset(&pak_gN, 0, sizeof(pak_gN));
		memset(&pak_iN, 0, sizeof(pak_iN));
		memset(&pak_d1, 0, sizeof(pak_d1));
		memset(&pak_d2, 0, sizeof(pak_d2));
		memset(&pak_sT, 0, sizeof(pak_sT));
		memset(&pak_o1, 0, sizeof(pak_o1));
		memset(&rtk_debug1, 0, sizeof(rtk_debug1));
		memset(&gnss_integ, 0, sizeof(gnss_integ));
		memset(&ins_integ, 0, sizeof(ins_integ));
		memset(&monitor, 0, sizeof(monitor));
		memset(&gnss_kml, 0, sizeof(gnss_kml));
		memset(&ins_kml, 0, sizeof(ins_kml));

		all_type_pack_num[em_s1] = 0;
		all_type_pack_num[em_s2] = 0;
		all_type_pack_num[em_gN] = 0;
		all_type_pack_num[em_iN] = 0;
		all_type_pack_num[em_d1] = 0;
		all_type_pack_num[em_d2] = 0;
		all_type_pack_num[em_sT] = 0;
		all_type_pack_num[em_o1] = 0;
		all_type_pack_num[em_fM] = 0;
		all_type_pack_num[em_rt] = 0;
		all_type_pack_num[em_sP] = 0;
		all_type_pack_num[em_sV] = 0;
		all_type_pack_num[em_r1] = 0;
		all_type_pack_num[em_w1] = 0;
		all_type_pack_num[em_gI] = 0;
		all_type_pack_num[em_iI] = 0;
		all_type_pack_num[em_g1] = 0;
		all_type_pack_num[em_RM] = 0;

		all_type_file_output[em_s1] = 1;
		all_type_file_output[em_s2] = 1;
		all_type_file_output[em_gN] = 1;
		all_type_file_output[em_iN] = 1;
		all_type_file_output[em_d1] = 1;
		all_type_file_output[em_d2] = 1;
		all_type_file_output[em_sT] = 1;
		all_type_file_output[em_o1] = 1;
		all_type_file_output[em_fM] = 1;
		all_type_file_output[em_rt] = 1;
		all_type_file_output[em_sP] = 1;
		all_type_file_output[em_sV] = 1;
		all_type_file_output[em_r1] = 1;
		all_type_file_output[em_w1] = 1;
		all_type_file_output[em_gI] = 1;
		all_type_file_output[em_iI] = 1;
		all_type_file_output[em_g1] = 1;
		all_type_file_output[em_RM] = 1;
	}

	void Rtk330la_decoder::set_base_file_name(char * file_name)
	{
		strcpy(base_file_name, file_name);
	}

	void Rtk330la_decoder::append_early_gnss_kml() {
		gnss_kml.gps_week = pak_gN_early.GPS_Week;
		gnss_kml.gps_secs = pak_gN_early.GPS_TimeOfWeek;
		gnss_kml.position_type = pak_gN_early.positionMode;
		gnss_kml.latitude = (double)pak_gN_early.latitude*180.0 / MAX_INT;
		gnss_kml.longitude = (double)pak_gN_early.longitude*180.0 / MAX_INT;
		gnss_kml.height = pak_gN_early.height;
		gnss_kml.north_vel = (float)pak_gN_early.velocityNorth / 100.0f;
		gnss_kml.east_vel = (float)pak_gN_early.velocityEast / 100.0f;
		gnss_kml.up_vel = (float)pak_gN_early.velocityUp / 100.0f;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}

	void Rtk330la_decoder::append_gnss_kml() {
		gnss_kml.gps_week = pak_gN.GPS_Week;
		gnss_kml.gps_secs = pak_gN.GPS_TimeOfWeek;
		gnss_kml.position_type = pak_gN.positionMode;
		gnss_kml.latitude = (double)pak_gN.latitude*180.0 / MAX_INT;
		gnss_kml.longitude = (double)pak_gN.longitude*180.0 / MAX_INT;
		gnss_kml.height = pak_gN.height;
		gnss_kml.north_vel = (float)pak_gN.velocityNorth / 100.0f;
		gnss_kml.east_vel = (float)pak_gN.velocityEast / 100.0f;
		gnss_kml.up_vel = (float)pak_gN.velocityUp / 100.0f;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}

	void Rtk330la_decoder::append_ins_kml() {
		ins_kml.gps_week = pak_iN.GPS_Week;
		ins_kml.gps_secs = pak_iN.GPS_TimeOfWeek;
		ins_kml.ins_status = pak_iN.insStatus;
		ins_kml.ins_position_type = pak_iN.insPositionType;
		ins_kml.latitude = (double)pak_iN.latitude*180.0 / MAX_INT;
		ins_kml.longitude = (double)pak_iN.longitude*180.0 / MAX_INT;
		ins_kml.height = pak_iN.height;
		ins_kml.north_velocity = (float)pak_iN.velocityNorth / 100.0f;
		ins_kml.east_velocity = (float)pak_iN.velocityEast / 100.0f;
		ins_kml.up_velocity = (float)pak_iN.velocityUp / 100.0f;
		ins_kml.roll = (float)pak_iN.roll / 100.0f;
		ins_kml.pitch = (float)pak_iN.pitch / 100.0f;
		ins_kml.heading = (float)pak_iN.heading / 100.0f;
		Kml_Generator::Instance()->append_ins(ins_kml);
	}

	void Rtk330la_decoder::output_s1() {
		//csv
		std::string name = "s1.csv";
		std::string title = 
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2)"
			",x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n";
		FILE* fp = get_file(name, title);
		if (fp) {
			fprintf(fp, "%d,%11.4f", pak_s1.GPS_Week, pak_s1.GPS_TimeOfWeek);
			fprintf(fp, ",%14.10f,%14.10f,%14.10f", pak_s1.x_accel, pak_s1.y_accel, pak_s1.z_accel);
			fprintf(fp, ",%14.10f,%14.10f,%14.10f\n", pak_s1.x_gyro, pak_s1.y_gyro, pak_s1.z_gyro);
		}
	}

	void Rtk330la_decoder::output_s2() {
		//csv
		std::string name_csv = "s2.csv";
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2)"
			",x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n";
		FILE* fp_csv = get_file(name_csv, title);
		sprintf(output_msg,
			"%d,%11.4f"
			",%14.10f,%14.10f,%14.10f"
			",%14.10f,%14.10f,%14.10f\n",
			pak_s2.GPS_Week, pak_s2.GPS_TimeOfWeek,
			pak_s2.x_accel, pak_s2.y_accel, pak_s2.z_accel,
			pak_s2.x_gyro, pak_s2.y_gyro, pak_s2.z_gyro);
		if (fp_csv) fprintf(fp_csv, output_msg);
		//txt
		FILE* fp_txt = get_file("imu.txt");
		sprintf(output_msg,
			"%d,%11.4f,    "
			",%14.10f,%14.10f,%14.10f"
			",%14.10f,%14.10f,%14.10f\n",
			pak_s2.GPS_Week, pak_s2.GPS_TimeOfWeek,
			pak_s2.x_accel, pak_s2.y_accel, pak_s2.z_accel,
			pak_s2.x_gyro, pak_s2.y_gyro, pak_s2.z_gyro);
		if (fp_txt) fprintf(fp_txt, output_msg);
		//process
		FILE* fp_process = get_file("process");
		if (fp_process) fprintf(fp_process, "$GPIMU,%s",output_msg);
	}

	void Rtk330la_decoder::output_gN_early() {
		float north_vel = (float)pak_gN_early.velocityNorth / 100.0f;
		float east_vel = (float)pak_gN_early.velocityEast / 100.0f;
		float up_vel = (float)pak_gN_early.velocityUp / 100.0f;
		float latitude_std = (float)pak_gN_early.latitude_std / 1000.0f;
		float longitude_std = (float)pak_gN_early.longitude_std / 1000.0f;
		float height_std = (float)pak_gN_early.height_std / 1000.0f;
		double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
		double track_over_ground = atan2(east_vel, north_vel) * R2D;
		//csv
		std::string title =
			"GPS_Week(),GPS_TimeofWeek(s),positionMode()"
			",latitude(deg),longitude(deg),height(m)"
			",numberOfSVs(),hdop(),diffage()"
			",velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s)"
			",latitude_std(m),longitude_std(m),height_std(m)\n";
		FILE* f_gN = get_file("gN.csv", title);
		sprintf(output_msg, "%d,%11.4f,%3d,%14.9f,%14.9f,%10.4f,%3d,%5.1f,%5.1f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n",
			pak_gN_early.GPS_Week, pak_gN_early.GPS_TimeOfWeek,
			pak_gN_early.positionMode, (double)pak_gN_early.latitude * 180.0 / MAX_INT, (double)pak_gN_early.longitude * 180.0 / MAX_INT, pak_gN_early.height,
			pak_gN_early.numberOfSVs, pak_gN_early.hdop, (float)pak_gN_early.diffage,
			north_vel, east_vel, up_vel, latitude_std, longitude_std, height_std);
		if (f_gN) fprintf(f_gN, output_msg);
		//txt
		FILE* f_gnssposvel = get_file("gnssposvel.txt");
		sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			pak_gN_early.GPS_Week, pak_gN_early.GPS_TimeOfWeek, pak_gN_early.latitude * 180.0 / MAX_INT, pak_gN_early.longitude * 180.0 / MAX_INT, pak_gN_early.height,
			latitude_std, longitude_std, height_std, pak_gN_early.positionMode, north_vel, east_vel, up_vel, track_over_ground);
		if (f_gnssposvel) fprintf(f_gnssposvel, output_msg);
		//process $GPGNSS
		FILE* f_process = get_file("process");
		sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%3d\n"
			, pak_gN_early.GPS_Week, pak_gN_early.GPS_TimeOfWeek
			, (double)pak_gN_early.latitude * 180.0 / MAX_INT, (double)pak_gN_early.longitude * 180.0 / MAX_INT
			, pak_gN_early.height, latitude_std, longitude_std, height_std
			, pak_gN_early.positionMode, pak_gN_early.diffage);
		if (f_process) fprintf(f_process, "$GPGNSS,%s", output_msg);
		//process $GPVEL
		sprintf(output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n", pak_gN_early.GPS_Week, pak_gN_early.GPS_TimeOfWeek, horizontal_speed, track_over_ground, up_vel);
		if (f_process) fprintf(f_process, "$GPVEL,%s", output_msg);
		//kml
		append_early_gnss_kml();

		FILE* f_gnss_time_txt = get_file("gnss_process_time.csv");
		if(f_gnss_time_txt) fprintf(f_gnss_time_txt, "%d,%11.4f,%6.4f\n", pak_gN_early.GPS_Week, pak_gN_early.GPS_TimeOfWeek, (pak_s1.GPS_TimeOfWeek - pak_gN_early.GPS_TimeOfWeek));
	}

	void Rtk330la_decoder::output_gN() {
		//double span_time = 0;
		//if (last_GPS_TimeOfWeek != 0.0) {
		//	span_time = inceptio_pak_gN.GPS_TimeOfWeek - last_GPS_TimeOfWeek;
		//	if (span_time > 1) {
		//		fprintf(f_log, "%11.4f,%11.4f,%f \n", last_GPS_TimeOfWeek, inceptio_pak_gN.GPS_TimeOfWeek, span_time);
		//	}
		//}
		float north_vel = (float)pak_gN.velocityNorth / 100.0f;
		float east_vel = (float)pak_gN.velocityEast / 100.0f;
		float up_vel = (float)pak_gN.velocityUp / 100.0f;
		float latitude_std = (float)pak_d2.latitude_std / 100.0f;
		float longitude_std = (float)pak_d2.longitude_std / 100.0f;
		float height_std = (float)pak_d2.height_std / 100.0f;
		float pos_hor_pl = (float)pak_gN.pos_hor_pl / 1000.0f;
		float pos_ver_pl = (float)pak_gN.pos_ver_pl / 1000.0f;
		float vel_hor_pl = (float)pak_gN.vel_hor_pl / 1000.0f;
		float vel_ver_pl = (float)pak_gN.vel_ver_pl / 1000.0f;
		double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
		double track_over_ground = atan2(east_vel, north_vel) * R2D;
		//csv
		std::string title =
			"GPS_Week(),GPS_TimeofWeek(s),positionMode()"
			",latitude(deg),longitude(deg),height(m)"
			",numberOfSVs(),hdop(),vdop(),tdop(),diffage()"
			",velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s)"
			",latitude_std(m),longitude_std(m),height_std(m)"
			",pos_hor_pl(m),pos_ver_pl(m),pos_status()"
			",vel_hor_pl(m/s),vel_ver_pl(m/s),vel_status()\n";
		FILE* f_gN = get_file("gN.csv",title);
		FILE* f_gnss_and_integrity = get_file("gnss_and_integrity.csv", title);
		sprintf(output_msg,
			"%d,%11.4f,%3d"
			",%14.9f,%14.9f,%10.4f"
			",%3d,%5.1f,%5.1f,%5.1f,%5.1f"
			",%10.4f,%10.4f,%10.4f"
			",%10.4f,%10.4f,%10.4f"
			",%10.4f,%10.4f"
			",%3d,%10.4f,%10.4f,%3d\n"
			, pak_gN.GPS_Week, pak_gN.GPS_TimeOfWeek, pak_gN.positionMode
			, (double)pak_gN.latitude*180.0 / MAX_INT, (double)pak_gN.longitude*180.0 / MAX_INT, pak_gN.height
			, pak_gN.numberOfSVs, pak_gN.hdop, pak_gN.vdop, pak_gN.tdop, (float)pak_gN.diffage
			, north_vel, east_vel, up_vel
			, latitude_std, longitude_std, height_std
			, pos_hor_pl, pos_ver_pl
			, pak_gN.pos_status, vel_hor_pl, vel_ver_pl, pak_gN.vel_status);
		if (f_gN) fprintf(f_gN, output_msg);
		if (f_gnss_and_integrity) fprintf(f_gnss_and_integrity, output_msg);
		//txt
		FILE* f_gnssposvel = get_file("gnssposvel.txt");
		sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			pak_gN.GPS_Week, pak_gN.GPS_TimeOfWeek, pak_gN.latitude*180.0 / MAX_INT, pak_gN.longitude*180.0 / MAX_INT, pak_gN.height,
			latitude_std, longitude_std, height_std, pak_gN.positionMode, north_vel, east_vel, up_vel, track_over_ground);
		if (f_gnssposvel) fprintf(f_gnssposvel, output_msg);
		//process $GPGNSS
		FILE* f_process = get_file("process");
		sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%3d\n"
			, pak_gN.GPS_Week, pak_gN.GPS_TimeOfWeek
			, (double)pak_gN.latitude*180.0 / MAX_INT, (double)pak_gN.longitude*180.0 / MAX_INT
			, pak_gN.height, latitude_std, longitude_std, height_std
			, pak_gN.positionMode, pak_gN.diffage);
		if (f_process) fprintf(f_process, "$GPGNSS,%s", output_msg);
		//process $GPVEL
		sprintf(output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n", pak_gN.GPS_Week, pak_gN.GPS_TimeOfWeek, horizontal_speed, track_over_ground, up_vel);
		if (f_process) fprintf(f_process, "$GPVEL,%s", output_msg);
		//kml
		append_gnss_kml();
		//time
		FILE* f_gnss_time_txt = get_file("gnss_process_time.csv");
		if (f_gnss_time_txt) fprintf(f_gnss_time_txt, "%d,%11.4f,%6.4f\n", pak_gN.GPS_Week, pak_gN.GPS_TimeOfWeek, (pak_s2.GPS_TimeOfWeek - pak_gN.GPS_TimeOfWeek));
		//last_GPS_TimeOfWeek = inceptio_pak_gN.GPS_TimeOfWeek;
	}

	void Rtk330la_decoder::output_iN() {
		//csv
		std::string title = 
			"GPS_Week(),GPS_TimeofWeek(s)"
			",insStatus(),insPositionType()"
			",latitude(deg),longitude(deg),height(m)"
			",velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s)"
			",roll(deg),pitch(deg),heading(deg)\n";
		FILE* f_iN = get_file("iN.csv", title);
		sprintf(output_msg, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f\n", pak_iN.GPS_Week, pak_iN.GPS_TimeOfWeek,
			pak_iN.insStatus, pak_iN.insPositionType,
			(double)pak_iN.latitude*180.0 / MAX_INT, (double)pak_iN.longitude*180.0 / MAX_INT, pak_iN.height,
			(float)pak_iN.velocityNorth / 100.0, (float)pak_iN.velocityEast / 100.0, (float)pak_iN.velocityUp / 100.0,
			(float)pak_iN.roll / 100.0, (float)pak_iN.pitch / 100.0, (float)pak_iN.heading / 100.0);
		if (f_iN) fprintf(f_iN, output_msg);
		uint32_t GPS_TimeOfWeek = (uint32_t)(pak_iN.GPS_TimeOfWeek * 100) * 10;
		if (GPS_TimeOfWeek % 100 == 0) {
			//txt
			FILE* f_ins = get_file("ins.txt");			
			sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n", pak_iN.GPS_Week, pak_iN.GPS_TimeOfWeek,
				(double)pak_iN.latitude*180.0 / MAX_INT, (double)pak_iN.longitude*180.0 / MAX_INT, pak_iN.height,
				(float)pak_iN.velocityNorth / 100.0, (float)pak_iN.velocityEast / 100.0, (float)pak_iN.velocityUp / 100.0,
				(float)pak_iN.roll / 100.0, (float)pak_iN.pitch / 100.0, (float)pak_iN.heading / 100.0, pak_iN.insPositionType, pak_iN.insStatus);
			if (f_ins) fprintf(f_ins, output_msg);
			//process
			FILE* f_process = get_file("process");
			sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d\n", pak_iN.GPS_Week, pak_iN.GPS_TimeOfWeek,
				(double)pak_iN.latitude*180.0 / MAX_INT, (double)pak_iN.longitude*180.0 / MAX_INT, pak_iN.height,
				(float)pak_iN.velocityNorth / 100.0, (float)pak_iN.velocityEast / 100.0, (float)pak_iN.velocityUp / 100.0,
				(float)pak_iN.roll / 100.0, (float)pak_iN.pitch / 100.0, (float)pak_iN.heading / 100.0, pak_iN.insPositionType);
			if (f_process) fprintf(f_process, "$GPINS,%s", output_msg);
		}
		//kml
		append_ins_kml();
	}

	void Rtk330la_decoder::output_d1() {
		//csv
		std::string title = 
			"GPS_Week(),GPS_TimeofWeek(s)"
			",latitude_std(),longitude_std(),height_std()"
			",north_vel_std(),east_vel_std(),up_vel_std()"
			",roll_std(),pitch_std(),heading_std()\n";
		FILE* f_d1 = get_file("d1.csv", title);
		sprintf(output_msg, "%d,%11.4f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n"
			, pak_d1.GPS_Week, pak_d1.GPS_TimeOfWeek
			, (float)pak_d1.latitude_std / 100.0, (float)pak_d1.longitude_std / 100.0, (float)pak_d1.height_std / 100.0
			, (float)pak_d1.north_vel_std / 100.0, (float)pak_d1.east_vel_std / 100.0, (float)pak_d1.up_vel_std / 100.0
			, (float)pak_d1.roll_std / 100.0, (float)pak_d1.pitch_std / 100.0, (float)pak_d1.heading_std / 100.0);
		if (f_d1) fprintf(f_d1, output_msg);
	}

	void Rtk330la_decoder::output_d2() {
		//csv
		std::string title =
			"GPS_Week(),GPS_TimeofWeek(s)"
			",latitude_std(),longitude_std(),height_std()"
			",north_vel_std(),east_vel_std(),up_vel_std()\n";
		FILE* f_d2 = get_file("d2.csv", title);
		sprintf(output_msg, "%d,%11.4f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n", pak_d2.GPS_Week, pak_d2.GPS_TimeOfWeek,
			(float)pak_d2.latitude_std / 100.0, (float)pak_d2.longitude_std / 100.0, (float)pak_d2.height_std / 100.0,
			(float)pak_d2.north_vel_std / 100.0, (float)pak_d2.east_vel_std / 100.0, (float)pak_d2.up_vel_std / 100.0);
		if (f_d2) fprintf(f_d2, output_msg);
	}

	void Rtk330la_decoder::output_sT() {
		//csv
		std::string title = 
			"GPS_Week(),GPS_TimeofWeek(s),year(),mouth(),day(),hour(),min(),sec()"
			",imu_temp_status,imu_acce_status,imu_gyro_status"
			",imu_sensor_status1,imu_sensor_status2,imu_sensor_status3"
			",imu_overall_status,gnss_data_status,gnss_signal_status"
			",power,MCU_status,pps_status,zupt_det,odo_used,odo_recv"
			",imu_s1_state,imu_s2_state,imu_s3_state,time_valid,antenna_sensing"
			",gnss_chipset,pust_check,imu_temperature,mcu_temperature\n";
		FILE* f_sT = get_file("sT.csv", title);
		sprintf(output_msg, "%d,%11.4f"
			",%5d,%5d,%5d,%5d,%5d,%5d"
			",%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d"
			",%8.3f,%8.3f\n"
			, pak_sT.GPS_Week, pak_sT.GPS_TimeOfWeek,
			pak_sT.year, pak_sT.mouth, pak_sT.day, pak_sT.hour, pak_sT.min, pak_sT.sec,
			pak_sT.status_bit.imu_temp_status, pak_sT.status_bit.imu_acce_status, pak_sT.status_bit.imu_gyro_status,
			pak_sT.status_bit.imu_sensor_status1, pak_sT.status_bit.imu_sensor_status2, pak_sT.status_bit.imu_sensor_status3, pak_sT.status_bit.imu_overall_status,
			pak_sT.status_bit.gnss_data_status, pak_sT.status_bit.gnss_signal_status, pak_sT.status_bit.power, pak_sT.status_bit.MCU_status, pak_sT.status_bit.pps_status,
			pak_sT.status_bit.zupt_det, pak_sT.status_bit.odo_used, pak_sT.status_bit.odo_recv,
			pak_sT.status_bit.imu_s1_state, pak_sT.status_bit.imu_s2_state, pak_sT.status_bit.imu_s3_state,
			pak_sT.status_bit.time_valid, pak_sT.status_bit.antenna_sensing, pak_sT.status_bit.gnss_chipset,
			pak_sT.status_bit.pust_check,
			pak_sT.imu_temperature, pak_sT.mcu_temperature);
		if (f_sT) fprintf(f_sT, output_msg);
	}

	void Rtk330la_decoder::output_debug1() {
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s),INS_AID()\n";
		FILE* f_r1 = get_file("r1.csv", title);
		sprintf(output_msg, "%d,%11.4f,%d\n", rtk_debug1.gps_week, rtk_debug1.gps_millisecs, rtk_debug1.ins_aid);
		if (f_r1) fprintf(f_r1, output_msg);
	}

	void Rtk330la_decoder::output_o1() {
		//csv
		std::string title = 
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",mode(),speed(m/s),fwd(),wheel_tick()\n";
		FILE* f_o1 = get_file("o1.csv", title);
		sprintf(output_msg, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", pak_o1.GPS_Week, (double)pak_o1.GPS_TimeOfWeek / 1000.0, pak_o1.mode,
			pak_o1.speed, pak_o1.fwd, pak_o1.wheel_tick);
		if (f_o1) fprintf(f_o1, output_msg);
		//txt
		FILE* f_odo = get_file("odo.txt", title);
		if (f_odo) fprintf(f_odo, output_msg);
		//process
		FILE* f_process = get_file("process");
		if (f_process) fprintf(f_process, "$GPODO,%s", output_msg);
	}

	void Rtk330la_decoder::output_gnss_integ()
	{
		std::string title =
			"week,timeOfWeek(s),spp_fail_rate,rtk_fail_rate"
			",spp_hor_pos_pl(m),spp_ver_pos_pl(m),spp_hor_vel_pl(m/s),spp_ver_vel_pl(m/s)"
			",rtk_hor_pos_pl(m),rtk_ver_pos_pl(m),rtk_hor_vel_pl(m/s),rtk_ver_vel_pl(m/s),rtk_heading_pl(deg)"
			",spp_hor_pos_al(m),spp_ver_pos_al(m),spp_hor_vel_al(m/s),spp_ver_vel_al(m/s)"
			",rtk_hor_pos_al(m),rtk_ver_pos_al(m),rtk_hor_vel_al(m/s),rtk_ver_vel_al(m/s),rtk_heading_pl(deg)"
			",spp_hor_pos_stat,spp_ver_pos_stat,spp_hor_vel_stat,spp_ver_vel_stat"
			",rtk_hor_pos_stat,rtk_ver_pos_stat,rtk_hor_vel_stat,rtk_ver_vel_stat,rtk_heading_stat\n";
		FILE* f_gnss_integ = get_file("gnss_intergrity.csv", title);
		if (f_gnss_integ) {
			fprintf(f_gnss_integ, "%4d,%11.4f,%8.3f,%8.3f", gnss_integ.gps_week, (double)gnss_integ.gps_millisecs / 1000, (float)gnss_integ.spp_fail_rate / 1.0e-10, (float)gnss_integ.rtk_fail_rate / 1.0e-10);
			fprintf(f_gnss_integ, ",%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.spp_hor_pos_pl / 100, (float)gnss_integ.spp_ver_pos_pl / 100, (float)gnss_integ.spp_hor_vel_pl / 100, (float)gnss_integ.spp_ver_vel_pl / 100);
			fprintf(f_gnss_integ, ",%8.3f,%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.rtk_hor_pos_pl / 100, (float)gnss_integ.rtk_ver_pos_pl / 100, (float)gnss_integ.rtk_hor_vel_pl / 100, (float)gnss_integ.rtk_ver_vel_pl / 100, (float)gnss_integ.rtk_heading_pl / 100);
			fprintf(f_gnss_integ, ",%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.spp_hor_pos_al / 100, (float)gnss_integ.spp_ver_pos_al / 100, (float)gnss_integ.spp_hor_vel_al / 100, (float)gnss_integ.spp_ver_vel_al / 100);
			fprintf(f_gnss_integ, ",%8.3f,%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.rtk_hor_pos_al / 100, (float)gnss_integ.rtk_ver_pos_al / 100, (float)gnss_integ.rtk_hor_vel_al / 100, (float)gnss_integ.rtk_ver_vel_al / 100, (float)gnss_integ.rtk_heading_al / 100);
			fprintf(f_gnss_integ, ",%2d,%2d,%2d,%2d", gnss_integ.status_bit.spp_hor_pos_s, gnss_integ.status_bit.spp_ver_pos_s, gnss_integ.status_bit.spp_hor_vel_s, gnss_integ.status_bit.spp_ver_vel_s);
			fprintf(f_gnss_integ, ",%2d,%2d,%2d,%2d,%2d", gnss_integ.status_bit.rtk_hor_pos_s, gnss_integ.status_bit.rtk_ver_pos_s, gnss_integ.status_bit.rtk_hor_vel_s, gnss_integ.status_bit.rtk_ver_vel_s, gnss_integ.status_bit.rtk_heading_s);
			fprintf(f_gnss_integ, "\n");
		}
	}

	void Rtk330la_decoder::output_ins_integ()
	{
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",hor_pos_pl,ver_pos_pl,hor_vel_pl,ver_vel_pl"
			",pitch_pl,roll_pl,heading_pl"
			",hor_pos_pl_status,ver_pos_pl_status,hor_vel_pl_status,ver_vel_pl_status"
			",pitch_pl_status,roll_pl_status,heading_pl_status\n";
		FILE* f_ins_integ = get_file("ins_intergrity.csv", title);
		if (f_ins_integ) {
			fprintf(f_ins_integ, "%4d,%11.4f", ins_integ.gps_week, (double)ins_integ.gps_millisecs / 1000.0);
			fprintf(f_ins_integ, ",%8.3f,%8.3f,%8.3f,%8.3f", ins_integ.hor_pos_pl, ins_integ.ver_pos_pl, ins_integ.hor_vel_pl, ins_integ.ver_vel_pl);
			fprintf(f_ins_integ, ",%8.3f,%8.3f,%8.3f", ins_integ.pitch_pl, ins_integ.roll_pl, ins_integ.heading_pl);
			fprintf(f_ins_integ, ",%2d,%2d,%2d,%2d", ins_integ.hor_pos_pl_status, ins_integ.ver_pos_pl_status, ins_integ.hor_vel_pl_status, ins_integ.ver_vel_pl_status);
			fprintf(f_ins_integ, ",%2d,%2d,%2d", ins_integ.pitch_pl_status, ins_integ.roll_pl_status, ins_integ.heading_pl_status);
			fprintf(f_ins_integ, "\n");
		}
	}

	void Rtk330la_decoder::output_ins_and_integ()
	{
		if (ins_integ.gps_millisecs % 100 != 0) {
			return;
		}
		if (uint32_t(pak_iN.GPS_TimeOfWeek * 1000) != ins_integ.gps_millisecs) {
			return;
		}
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",latitude(deg),longitude(deg),height(m)"
			",north_velocity(m/s),east_velocity(m/s),up_velocity(m/s)"
			",roll(deg),pitch(deg),heading(deg)"
			",ins_status(),ins_position_type()"
			",latitude_std(m),longitude_std(m),height_std(m)"
			",north_velocity_std(m/s),east_velocity_std(m/s),up_velocity_std(m/s)"
			",roll_std(deg),pitch_std(deg),heading_std(deg)"
			",hor_pos_pl,ver_pos_pl,hor_vel_pl,ver_vel_pl"
			",pitch_pl,roll_pl,heading_pl\n";
		FILE* f_ins_and_integ = get_file("ins_and_intergrity.csv", title);
		if (f_ins_and_integ) {
			fprintf(f_ins_and_integ, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d", pak_iN.GPS_Week, pak_iN.GPS_TimeOfWeek,
				(double)pak_iN.latitude*180.0 / MAX_INT, (double)pak_iN.longitude*180.0 / MAX_INT, pak_iN.height,
				(float)pak_iN.velocityNorth / 100.0, (float)pak_iN.velocityEast / 100.0, (float)pak_iN.velocityUp / 100.0,
				(float)pak_iN.roll / 100.0, (float)pak_iN.pitch / 100.0, (float)pak_iN.heading / 100.0, pak_iN.insPositionType, pak_iN.insStatus);
			fprintf(f_ins_and_integ, ",%8.3f,%8.3f,%8.3f", (float)pak_d1.latitude_std / 100.0, (float)pak_d1.longitude_std / 100.0, (float)pak_d1.height_std / 100.0);
			fprintf(f_ins_and_integ, ",%8.3f,%8.3f,%8.3f", (float)pak_d1.north_vel_std / 100.0, (float)pak_d1.east_vel_std / 100.0, (float)pak_d1.up_vel_std / 100.0);
			fprintf(f_ins_and_integ, ",%8.3f,%8.3f,%8.3f", (float)pak_d1.roll_std / 100.0, (float)pak_d1.pitch_std / 100.0, (float)pak_d1.heading_std / 100.0);
			fprintf(f_ins_and_integ, ",%8.3f,%8.3f,%8.3f,%8.3f", ins_integ.hor_pos_pl, ins_integ.ver_pos_pl, ins_integ.hor_vel_pl, ins_integ.ver_vel_pl);
			fprintf(f_ins_and_integ, ",%8.3f,%8.3f,%8.3f\n", ins_integ.pitch_pl, ins_integ.roll_pl, ins_integ.heading_pl);
		}
	}

	void Rtk330la_decoder::output_g1() {
		uint8_t* payload = raw.buff + 3;
		FILE* f_g1 = get_file("g1.csv");
		if (f_g1) fwrite(payload, 1, raw.length, f_g1);
	}

	void Rtk330la_decoder::output_runstatus_monitor()
	{
		uint8_t* payload = raw.buff + 3;
		uint8_t* offset = payload;
		size_t header_size = sizeof(monitor.head);
		if (raw.length <= header_size)  return;

		memcpy(&monitor.head, offset, header_size);
		offset += header_size;

		if (monitor.head.report_mask & (1 << 0)) {
			size_t pak_size = sizeof(monitor.imu);
			memcpy(&monitor.imu, offset, pak_size);
			offset += pak_size;
			std::string file_name = "monitor_imu.csv";
			std::string title = "week,tow,rollcnt,state,data_tow,receive_delay,master_status,temperature\n";
			FILE* file = get_file(file_name, title);
			if (file) {
				fprintf(file, "%d,%12.4f,", monitor.head.week, (double)monitor.head.tow / 1000.0);
				fprintf(file, "%7d,%6d,%12.4f,%7.3f,%5d,%7.2f\n",
					monitor.head.rollcnt, monitor.imu.state, (double)monitor.imu.data_tow / 1000.0,
					(double)monitor.imu.receive_delay / 1000.0, monitor.imu.master_status,
					(double)monitor.imu.temperature / 100.0);
			}
		}
		if (monitor.head.report_mask & (1 << 1)) {
			size_t pak_size = sizeof(monitor.TeseoV);
			memcpy(&monitor.TeseoV, offset, pak_size);
			offset += pak_size;
			std::string file_name = "monitor_TeseoV.csv";
			std::string title = "week,tow,state,data_tow,safe_state,PPS_status,time_validity,system_status,antenna_sensing,temperature,CPU_usage,epvt_status,RTCM_obs,PPS_cnt,PPS_measurement\n";
			FILE* file = get_file(file_name, title);
			if (file) {
				fprintf(file, "%d,%12.4f,", monitor.head.week, (double)monitor.head.tow / 1000);
				fprintf(file, "%6d,%12.4f,%4d,%4d,%4d,%4d,%4d,%7.2f,%7.2f,%4d,%11d,%11d,%11d\n",
					monitor.TeseoV.state, (double)monitor.TeseoV.data_tow / 1000,
					monitor.TeseoV.safe_state, monitor.TeseoV.PPS_status,
					monitor.TeseoV.time_validity, monitor.TeseoV.system_status,
					monitor.TeseoV.antenna_sensing, (double)monitor.TeseoV.temperature / 100,
					(double)monitor.TeseoV.CPU_usage / 100, monitor.TeseoV.epvt_status,
					monitor.TeseoV.RTCM_obs, monitor.TeseoV.PPS_cnt, monitor.TeseoV.PPS_measurement);
			}
		}
		if (monitor.head.report_mask & (1 << 2)) {
			size_t pak_size = sizeof(monitor.base);
			memcpy(&monitor.base, offset, pak_size);
			offset += pak_size;
			std::string file_name = "monitor_base.csv";
			std::string title = "week,tow,obs\n";
			FILE* file = get_file(file_name, title);
			if (file) {
				fprintf(file, "%d,%12.4f,", monitor.head.week, (double)monitor.head.tow / 1000);
				fprintf(file, "%11d\n", monitor.base.RTCM_obs);
			}

		}
		if (monitor.head.report_mask & (1 << 3)) {
			size_t pak_size = sizeof(monitor.gnss);
			memcpy(&monitor.gnss, offset, pak_size);
			offset += pak_size;
			std::string file_name = "monitor_gnss.csv";
			std::string title = "week,tow,sol_tow,fixtype,delay\n";
			FILE* file = get_file(file_name, title);
			if (file) {
				fprintf(file, "%d,%12.4f,", monitor.head.week, (double)monitor.head.tow / 1000);
				fprintf(file, "%13.4f,%5d,%7d\n", (double)monitor.gnss.sol_tow / 1000, monitor.gnss.sol_fixtype, monitor.gnss.sol_delay);
			}
		}
		if (monitor.head.report_mask & (1 << 4)) {
			size_t pak_size = sizeof(monitor.odo);
			memcpy(&monitor.odo, offset, pak_size);
			offset += pak_size;
			std::string file_name = "monitor_odo.csv";
			std::string title = "week,tow,odo_tow\n";
			FILE* file = get_file(file_name, title);
			if (file) {
				fprintf(file, "%d,%12.4f,", monitor.head.week, (double)monitor.head.tow / 1000);
				fprintf(file, "%13.4f\n", (double)monitor.odo.odo_tow / 1000);
			}
		}
		if (monitor.head.report_mask & (1 << 5)) {
			size_t pak_size = sizeof(monitor.ins);
			memcpy(&monitor.ins, offset, pak_size);
			offset += pak_size;
			std::string file_name = "monitor_ins.csv";
			std::string title = "week,tow,status,position_type,sol_delay\n";
			FILE* file = get_file(file_name, title);
			if (file) {
				fprintf(file, "%d,%12.4f,", monitor.head.week, (double)monitor.head.tow / 1000);
				fprintf(file, "%5d,%5d,%7.3f\n", monitor.ins.status, monitor.ins.position_type, (double)monitor.ins.sol_delay / 1000);
			}
		}
		if (monitor.head.report_mask & (1 << 6)) {
			size_t pak_size = sizeof(monitor.sys);
			memcpy(&monitor.sys, offset, pak_size);
			offset += pak_size;
			std::string file_name = "monitor_sys.csv";
			std::string title = "week,tow,system_time_status,MCU_temperature,cycle_time\n";
			FILE* file = get_file(file_name, title);
			if (file) {
				fprintf(file, "%d,%12.4f,", monitor.head.week, (double)monitor.head.tow / 1000);
				fprintf(file, "%5d,%7.2f,%7.3f\n", monitor.sys.system_time_status, (double)monitor.sys.MCU_temperature / 100, (double)monitor.sys.cycle_time / 1000);
			}
		}
	}

	void Rtk330la_decoder::parse_packet_payload()
	{
		uint8_t* payload = raw.buff + 3;
		if (all_type_file_output.find(raw.packet_type) == all_type_file_output.end()) return;
		if (all_type_file_output[raw.packet_type] == 0) return;
		switch (raw.packet_type) {
		case em_s1:
		{
			size_t packet_size = sizeof(inceptio_s1_t);
			if (raw.length == packet_size) {
				memcpy(&pak_s1, payload, sizeof(inceptio_s1_t));
				if (!m_isOutputFile) break;
				output_s1();
			}

		}break;
		case em_s2:
		{
			size_t packet_size = sizeof(inceptio_s1_t);
			if (raw.length == packet_size) {
				memcpy(&pak_s2, payload, sizeof(inceptio_s1_t));
				if (!m_isOutputFile) break;
				output_s2();
#ifdef OUTPUT_INNER_FILE
				save_novatel_raw_imu();
#endif
			}

		}break;
		case em_gN:
		{
			if (raw.length == sizeof(inceptio_gN_more_early_t) 
				|| raw.length == sizeof(inceptio_gN_early_t)) {
				memcpy(&pak_gN_early, payload, raw.length);
				if (!m_isOutputFile) break;
				output_gN_early();
			}
			else if(raw.length == sizeof(inceptio_gN_24_01_21_t)
				|| raw.length == sizeof(inceptio_gN_t)){
				memcpy(&pak_gN, payload, raw.length);
				//output_gN();//em_d2
			}

		}break;
		case em_iN:
		{
			if (raw.length == sizeof(inceptio_iN_t)) {
				memcpy(&pak_iN, payload, sizeof(inceptio_iN_t));
				if (!m_isOutputFile) break;
				output_iN();
			}
		}break;
		case em_d1:
		{
			if (raw.length == sizeof(inceptio_d1_t)) {
				memcpy(&pak_d1, payload, sizeof(inceptio_d1_t));
				if (!m_isOutputFile) break;
				output_d1();
			}
		}break;
		case em_d2:
		{
			if (raw.length == sizeof(inceptio_d2_t)) {
				memcpy(&pak_d2, payload, sizeof(inceptio_d2_t));
				if (!m_isOutputFile) break;
				output_d2();
				output_gN();
			}
		}break;
		case em_sT:
		{
			if (raw.length == sizeof(inceptio_sT_t)) {
				memcpy(&pak_sT, payload, sizeof(inceptio_sT_t));
				if (!m_isOutputFile) break;
				output_sT();
			}
		}break;
		case em_o1:
		{
			if (raw.length == sizeof(inceptio_o1_t)) {
				memcpy(&pak_o1, payload, sizeof(inceptio_o1_t));
				if (!m_isOutputFile) break;
				output_o1();
			}
		}break;
		case em_fM:break;
		case em_rt:break;
		case em_sP:break;
		case em_sV:break;
		case em_r1:
		{
			if (raw.length == sizeof(rtk_debug1_t)) {
				memcpy(&rtk_debug1, payload, sizeof(rtk_debug1_t));
				if (!m_isOutputFile) break;
				output_debug1();
			}
		}break;
		case em_w1:break;
		case em_gI:
		{
			size_t psize = sizeof(gnss_integ_t);
			if (raw.length == psize) {
				memcpy(&gnss_integ, payload, psize);
				if (!m_isOutputFile) break;
				output_gnss_integ();
			}
		}break;
		case em_iI:
		{
			size_t psize = sizeof(ins_integ_t);
			if (raw.length == psize) {
				memcpy(&ins_integ, payload, psize);
				if (!m_isOutputFile) break;
				output_ins_integ();
				output_ins_and_integ();
			}
		}break;
		case em_g1: {
			if (!m_isOutputFile) break;
			output_g1();
		}break;
		case em_RM:
		{
			if (!m_isOutputFile) break;
			output_runstatus_monitor();
		}break;
		default:
			break;
		}
	}

	void Rtk330la_decoder::save_novatel_raw_imu() {
		FILE* f_imu_bin = get_file("raw_imu.bin");
		if (f_imu_bin) {
			novatel_rawimu_t raw_imu = { 0 };
			raw_imu.seconds = pak_s2.GPS_TimeOfWeek;
			raw_imu.x_gyro = pak_s2.x_gyro * Gyro_Scale_Factor / Date_Rate;
			raw_imu.y_gyro = pak_s2.y_gyro * Gyro_Scale_Factor / Date_Rate;
			raw_imu.z_gyro = pak_s2.z_gyro * Gyro_Scale_Factor / Date_Rate;
			raw_imu.x_accel = pak_s2.x_accel * Accel_Scale_Factor / Date_Rate;
			raw_imu.y_accel = pak_s2.y_accel * Accel_Scale_Factor / Date_Rate;
			raw_imu.z_accel = pak_s2.z_accel * Accel_Scale_Factor / Date_Rate;
			fwrite(&raw_imu, 1, sizeof(raw_imu), f_imu_bin);
		}
	}

	int Rtk330la_decoder::parse_nmea(uint8_t data) {
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
			if (is_nmea_char(data) && raw.nmeabyte < 256) {
				raw.nmea[raw.nmeabyte++] = data;
			}
			else {
				raw.nmea[raw.nmeabyte++] = 0x0A;
				raw.nmea[raw.nmeabyte++] = 0;
				raw.nmea_flag = 0;
				if (m_isOutputFile) {
					FILE* f_nmea = get_file("nmea.txt");
					fprintf(f_nmea, (char*)raw.nmea);
				}
				return 2;
			}
		}
		return 0;
	}

	int Rtk330la_decoder::input_raw(uint8_t data)
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
				std::map<uint16_t, int>::iterator it = all_type_pack_num.find(raw.packet_type);
				if (it != all_type_pack_num.end()) {
					raw.flag = 1;
					raw.buff[raw.nbyte++] = raw.header[2];
					raw.buff[raw.nbyte++] = raw.header[3];
				}
				raw.header_len = 0;
			}
			return parse_nmea(data);
		}
		else {
			raw.buff[raw.nbyte++] = data;
			if (raw.nbyte == 3) {
				raw.length = raw.buff[2];
			}
			if (raw.nbyte == raw.buff[2] + 5) { //5 = [type1,type2,len] + [crc1,crc2]
				uint16_t packet_crc = 256 * raw.buff[raw.nbyte - 2] + raw.buff[raw.nbyte - 1];
				uint16_t cal_crc = calc_crc(raw.buff, raw.nbyte - 2);
				pack_num++;
				all_type_pack_num[raw.packet_type]++;
				if (packet_crc == cal_crc) {
					crc_right_num++;
					parse_packet_payload();
					ret = 1;
				}
				else {
					crc_error_num++;
					//fprintf(f_log, "type=%c%c,crc=0x%04x:0x%04x,size=%d\n", raw.buff[0], raw.buff[1], packet_crc, cal_crc, raw.nbyte);
				}
				raw.flag = 0;
				raw.nbyte = 0;
				raw.length = 0;
			}
		}
		return ret;
	}

	void Rtk330la_decoder::finish()
	{
		if (!m_isOutputFile) return;
		FILE* f_log = get_file(".log");
		for (std::map<uint16_t, int>::iterator it = all_type_pack_num.begin(); it != all_type_pack_num.end(); it++) {
			fprintf(f_log, "pack_type = 0x%04x, pack_num = %d\n", (uint16_t)it->first, (int)it->second);
		}
		fprintf(f_log, "all_pack_num = %d\ncrc_right_num = %d\ncrc_error_num = %d\n", pack_num, crc_right_num, crc_error_num);
		Kml_Generator::Instance()->open_files(base_file_name);
		Kml_Generator::Instance()->write_files();
		Kml_Generator::Instance()->close_files();
		close_all_files();
	}

	int Rtk330la_decoder::get_current_type()
	{
		return raw.packet_type;
	}
	inceptio_s1_t * Rtk330la_decoder::get_imu_raw()
	{
		return &pak_s2;
	}
	inceptio_gN_t * Rtk330la_decoder::get_gnss_sol()
	{
		return &pak_gN;
	}
	inceptio_iN_t * Rtk330la_decoder::get_ins_sol()
	{
		return &pak_iN;
	}
}
#include "ins401.h"
#include <string.h>
#include <math.h>
#include "common.h"
#include "rtklib_core.h" //R2D
#include "rtkcmn.h"

namespace Ins401_Tool {

//#define MI_OUTPUT_FILE  //С������ļ�����Ŀ�����ж���

	Ins401_decoder::Ins401_decoder()
		:m_MI_file_switch(false)
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
		all_type_pack_num.clear();
		output_file_map.clear();
		init();
	}

	Ins401_decoder::~Ins401_decoder()
	{
		close_all_files();
	}

	void Ins401_decoder::init()
	{
		pack_num = 0;
		crc_right_num = 0;
		crc_error_num = 0;
		show_format_time = 0;
		height_msl = 0.0f;
		last_gnss_integ_millisecs = 0;
		memset(&raw, 0, sizeof(raw));
		memset(&imu, 0, sizeof(imu));
		memset(&gnss, 0, sizeof(gnss));
		memset(&ins, 0, sizeof(ins));
		memset(&odo, 0, sizeof(odo));
		memset(&misa, 0, sizeof(misa));
		memset(&dm, 0, sizeof(dm));
		memset(&powerup_dr, 0, sizeof(powerup_dr));
		memset(&gnss_integ, 0, sizeof(gnss_integ));
		memset(&rtk_debug1, 0, sizeof(rtk_debug1));
		memset(&system_fault_detection, 0, sizeof(system_fault_detection));		
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
		all_type_pack_num[PowerUpDR_MES] = 0;
		all_type_pack_num[em_CHECK] = 0;
		all_type_pack_num[em_GNSS_SOL_INTEGEITY] = 0;
		all_type_pack_num[em_RTK_DEBUG1] = 0;
		all_type_pack_num[em_PACKAGE_FD] = 0;
        all_type_pack_num[em_MOVBS_SOL] = 0;
		Kml_Generator::Instance()->init();
	}

	void Ins401_decoder::close_all_files()
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
		FilesMap::iterator it;
		for (it = output_file_map.begin(); it != output_file_map.end(); it++) {
			if(it->second) fclose(it->second); it->second = NULL;
		}
		output_file_map.clear();
	}

	void Ins401_decoder::create_file(FILE* &file, const char* suffix, const char* title, bool format_time = false) {
		if (strlen(base_file_name) == 0) return;
		if (file == NULL) {
			char file_name[256] = { 0 };
			sprintf(file_name, "%s_%s", base_file_name, suffix);
			file = fopen(file_name, "wb");
			if (file && title) {
				if(format_time)
					fprintf(file, "DateTime(),");
				fprintf(file, title);
			}
		}
	}

	char * Ins401_decoder::week_2_time_str(int week, uint32_t millisecs)
	{
		gtime_t gpstime = gpst2time(week, (double)millisecs / 1000.0);
		gtime_t utctime = gpst2utc(gpstime);
		return time_str(utctime, 2);
	}

	void Ins401_decoder::append_gnss_kml()
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

	void Ins401_decoder::append_ins_kml()
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

	void Ins401_decoder::output_imu_raw()
	{
		//csv
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2)"
			",x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n";
		create_file(f_imu_csv, "imu.csv", title.c_str(), show_format_time);
		if (show_format_time) {
			fprintf(f_imu_csv, "%s,",week_2_time_str(imu.gps_week, imu.gps_millisecs));
		}
		fprintf(f_imu_csv, "%d,%11.4f"
			",%14.10f,%14.10f,%14.10f"
			",%14.10f,%14.10f,%14.10f\n"
			, imu.gps_week, (double)imu.gps_millisecs / 1000.0
			, imu.accel_x, imu.accel_y, imu.accel_z
			, imu.gyro_x, imu.gyro_y, imu.gyro_z);
#ifdef OUTPUT_INNER_FILE
		//txt
		create_file(f_imu_txt, "imu.txt", NULL);
		fprintf(f_imu_txt, "%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n"
			,imu.gps_week, (double)imu.gps_millisecs / 1000.0
			,imu.accel_x, imu.accel_y, imu.accel_z
			,imu.gyro_x, imu.gyro_y, imu.gyro_z);
		//process
		create_file(f_process, "process", NULL);
		fprintf(f_process, "$GPIMU,%d,%11.4f,    ,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n"
			,imu.gps_week, (double)imu.gps_millisecs / 1000.0
			,imu.accel_x, imu.accel_y, imu.accel_z
			,imu.gyro_x, imu.gyro_y, imu.gyro_z);
#endif
	}

	void Ins401_decoder::MI_output_imu_raw()
	{
#ifdef MI_OUTPUT_FILE
		if (m_MI_file_switch == false) return;
		FILE* f_mi_imu_csv = NULL;
		std::string mi_imu_name = "MI-IMU.csv";
		FilesMap::iterator it = output_file_map.find(mi_imu_name);
		if (it == output_file_map.end()) {
			std::string mi_imu_title = "Tow,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,temper_imu\n";
			create_file(f_mi_imu_csv, mi_imu_name.c_str(), mi_imu_title.c_str());
			output_file_map[mi_imu_name] = f_mi_imu_csv;
		}
		f_mi_imu_csv = output_file_map[mi_imu_name];
		if (f_mi_imu_csv) {
			fprintf(f_mi_imu_csv, "%11.3f", (double)imu.gps_millisecs / 1000.0);
			fprintf(f_mi_imu_csv, ",%14.10f,%14.10f,%14.10f", imu.accel_x, imu.accel_y, imu.accel_z);
			fprintf(f_mi_imu_csv, ",%14.10f,%14.10f,%14.10f", imu.gyro_x*D2R, imu.gyro_y*D2R, imu.gyro_z*D2R);
			fprintf(f_mi_imu_csv, ",%5.1f\n", dm.IMU_Unit_temperature);
		}
#endif // MI_OUTPUT_FILE
	}

	void Ins401_decoder::output_gnss_sol()
	{
		//csv
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s),position_type()"
			",latitude(deg),longitude(deg),height(m)"
			",latitude_standard_deviation(m),longitude_standard_deviation(m),height_standard_deviation(m)"
			",number_of_satellites(),number_of_satellites_in_solution()"
			",hdop(),diffage(s)"
			",north_vel(m/s),east_vel(m/s),up_vel(m/s)"
			",north_vel_standard_deviation(m/s),east_vel_standard_deviation(m/s),up_vel_standard_deviation(m/s)\n";
		create_file(f_gnss_csv, "gnss.csv", title.c_str(), show_format_time);
		if (show_format_time) {
			fprintf(f_gnss_csv, "%s,", week_2_time_str(gnss.gps_week, gnss.gps_millisecs));
		}
		fprintf(f_gnss_csv, "%d,%11.4f,%3d", gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.position_type);
		fprintf(f_gnss_csv, ",%14.9f,%14.9f,%10.4f", gnss.latitude, gnss.longitude, gnss.height);
		fprintf(f_gnss_csv, ",%10.4f,%10.4f,%10.4f", gnss.latitude_std, gnss.longitude_std, gnss.height_std);
		fprintf(f_gnss_csv, ",%3d,%3d", gnss.numberOfSVs, gnss.numberOfSVs_in_solution);
		fprintf(f_gnss_csv, ",%5.1f,%5.1f", gnss.hdop, gnss.diffage);
		fprintf(f_gnss_csv, ",%10.4f,%10.4f,%10.4f", gnss.north_vel, gnss.east_vel, gnss.up_vel);
		fprintf(f_gnss_csv, ",%10.4f,%10.4f,%10.4f", gnss.north_vel_std, gnss.east_vel_std, gnss.up_vel_std);
		fprintf(f_gnss_csv, "\n");
#ifdef OUTPUT_INNER_FILE
		double track_over_ground = atan2(gnss.east_vel, gnss.north_vel)*R2D;
		double horizontal_speed = sqrt(gnss.north_vel * gnss.north_vel + gnss.east_vel * gnss.east_vel);
		//txt
		create_file(f_gnss_txt, "gnssposvel.txt", NULL);
		fprintf(f_gnss_txt, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.latitude, gnss.longitude, gnss.height,
			gnss.latitude_std, gnss.longitude_std, gnss.height_std,
			gnss.position_type, gnss.north_vel, gnss.east_vel, gnss.up_vel, track_over_ground);
		//process
		create_file(f_process, "process", NULL);
		//process $GPGNSS
		fprintf(f_process, "$GPGNSS,%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f\n",
			gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.latitude, gnss.longitude, gnss.height,
			gnss.latitude_std, gnss.longitude_std, gnss.height_std,
			gnss.position_type, gnss.diffage);
		//process $GPVEL
		fprintf(f_process, "$GPVEL,%d,%11.4f,%10.4f,%10.4f,%10.4f\n",
			gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, horizontal_speed, track_over_ground, gnss.up_vel);
#endif
		append_gnss_kml();
	}

	void Ins401_decoder::MI_output_gnss_sol()
	{
#ifdef MI_OUTPUT_FILE
		if (m_MI_file_switch == false) return;
		FILE* f_mi_gnss_csv = NULL;
		std::string mi_gnss_name = "MI-GNSS.csv";
		FilesMap::iterator it = output_file_map.find(mi_gnss_name);
		if (it == output_file_map.end()) {
			std::string mi_gnss_title = 
				"Tow,Status"
				",latitude,longitude,height"
				",VE,VN,VU"
				",heading_motion,heading_vehicle,roll,pitch"
				",std_latitude,std_longitude,std_height"
				",std_velocity_east,std_velocity_north,std_velocity_up"
				",std_heading_rad,std_roll_rad,std_pitch_rad,height_msl"
				",sv_num_used,cnr_mean,inno_pr_rms,inno_dr_rms"
				",Pdop,e_dop,n_dop,v_dop"
				",gnss_week,gnss_second_s"
				",utc_year,utc_month,utc_day,utc_hour,utc_min,utc_sec\n";
			create_file(f_mi_gnss_csv, mi_gnss_name.c_str(), mi_gnss_title.c_str());
			output_file_map[mi_gnss_name] = f_mi_gnss_csv;
		}
		f_mi_gnss_csv = output_file_map[mi_gnss_name];
		if (f_mi_gnss_csv) {
			double ep[6] = { 0 };
			gtime_t gtime = gst2time(gnss.gps_week, (double)gnss.gps_millisecs / 1000.0);
			double track_over_ground = atan2(gnss.east_vel, gnss.north_vel);
			if (track_over_ground < 0) {
				track_over_ground += 2 * PI;
			}
			time2epoch(gtime, ep);
			fprintf(f_mi_gnss_csv, "%11.3f,%3d", (double)gnss.gps_millisecs / 1000.0, gnss.position_type);
			fprintf(f_mi_gnss_csv, ",%14.9f,%14.9f,%10.4f", gnss.latitude, gnss.longitude, gnss.height);
			fprintf(f_mi_gnss_csv, ",%10.4f,%10.4f,%10.4f", gnss.east_vel, gnss.north_vel, gnss.up_vel);
			fprintf(f_mi_gnss_csv, ",%10.4f,%s,%s,%s", track_over_ground, " ", " ", " ");/*,heading_vehicle,roll,pitch*/
			fprintf(f_mi_gnss_csv, ",%10.4f,%10.4f,%10.4f", gnss.latitude_std, gnss.longitude_std, gnss.height_std);
			fprintf(f_mi_gnss_csv, ",%10.4f,%10.4f,%10.4f", gnss.east_vel_std, gnss.north_vel_std, gnss.up_vel_std);
			fprintf(f_mi_gnss_csv, ",%s,%s,%s,%10.3f", " ", " ", " ", height_msl);/*,std_heading_rad,std_roll_rad,std_pitch_rad,height_msl*/
			fprintf(f_mi_gnss_csv, ",%3d,%s,%s,%s", gnss.numberOfSVs, " ", " ", " ");/*,cnr_mean,inno_pr_rms,inno_dr_rms*/
			fprintf(f_mi_gnss_csv, ",%5.1f,%s,%s,%s", gnss.hdop, " ", " ", " ");/*,Pdop,e_dop,n_dop,v_dop*/
			fprintf(f_mi_gnss_csv, ",%6d,%11.3f", gnss.gps_week, (double)gnss.gps_millisecs / 1000.0);
			fprintf(f_mi_gnss_csv, ",%4d,%02d,%02d,%02d,%02d,%04.1f", (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5]);
			fprintf(f_mi_gnss_csv, "\n");
		}
#endif // MI_OUTPUT_FILE
	}

	void Ins401_decoder::output_ins_sol() {
		//csv
		std::string title = 
			"GPS_Week(),GPS_TimeOfWeek(s),ins_status(),ins_position_type()"
			",latitude(deg),longitude(deg),height(m)"
			",north_velocity(m/s),east_velocity(m/s),up_velocity(m/s)"
			",longitudinal_velocity(m/s),lateral_velocity(m/s)"
			",roll(deg),pitch(deg),heading(deg)"
			",latitude_std(m),longitude_std(m),height_std(m)"
			",north_velocity_std(m/s),east_velocity_std(m/s),up_velocity_std(m/s)"
			",long_vel_std(m/s),lat_vel_std(m/s)"
			",roll_std(deg),pitch_std(deg),heading_std(deg)"
			",contient()\n";
		create_file(f_ins_csv, "ins.csv", title.c_str(), show_format_time);
		if (show_format_time) {
			fprintf(f_ins_csv, "%s,", week_2_time_str(ins.gps_week, ins.gps_millisecs));
		}
		fprintf(f_ins_csv, "%d,%11.4f,%3d,%3d,", ins.gps_week, (double)ins.gps_millisecs / 1000.0, ins.ins_status, ins.ins_position_type);
		fprintf(f_ins_csv, "%14.9f,%14.9f,%10.4f,", ins.latitude, ins.longitude, ins.height);
		fprintf(f_ins_csv, "%10.4f,%10.4f,%10.4f,", ins.north_velocity, ins.east_velocity, ins.up_velocity);
		fprintf(f_ins_csv, "%10.4f,%10.4f,", ins.longitudinal_velocity, ins.lateral_velocity);
		fprintf(f_ins_csv, "%14.9f,%14.9f,%14.9f,", ins.roll, ins.pitch, ins.heading);
		fprintf(f_ins_csv, "%8.3f,%8.3f,%8.3f,", ins.latitude_std, ins.longitude_std, ins.height_std);
		fprintf(f_ins_csv, "%8.3f,%8.3f,%8.3f,", ins.north_velocity_std, ins.east_velocity_std, ins.up_velocity_std);
		fprintf(f_ins_csv, "%8.3f,%8.3f,", ins.long_vel_std, ins.lat_vel_std);
		fprintf(f_ins_csv, "%8.3f,%8.3f,%8.3f,", ins.roll_std, ins.pitch_std, ins.heading_std);
		fprintf(f_ins_csv, "%3d\n", ins.id_contient);
#ifdef OUTPUT_INNER_FILE
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

	void Ins401_decoder::MI_output_ins_sol()
	{
#ifdef MI_OUTPUT_FILE
		if (m_MI_file_switch == false) return;
		FILE* f_mi_ins_csv = NULL;
		std::string ins_name = "MI-INS.csv";
		FilesMap::iterator it = output_file_map.find(ins_name);
		if (it == output_file_map.end()) {
			std::string mi_ins_title = 
				"Tow,Status"
				",latitude,longitude,height"
				",heading,roll_rad,pitch_rad"
				",VE,VN,VU"
				",acc_x,acc_y,acc_z"
				",gyr_x,gyr_y,gyr_z"
				",std_latitude,std_longitude,std_height"
				",std_velocity_east,std_velocity_north,std_velocity_up"
				",std_heading,std_roll,std_pitch"
				",height_msl\n";
			create_file(f_mi_ins_csv, ins_name.c_str(), mi_ins_title.c_str());
			output_file_map[ins_name] = f_mi_ins_csv;
		}
		f_mi_ins_csv = output_file_map[ins_name];
		if (f_mi_ins_csv) {
			fprintf(f_mi_ins_csv, "%11.3f,%3d", (double)ins.gps_millisecs / 1000.0, ins.ins_status);
			fprintf(f_mi_ins_csv, ",%14.9f,%14.9f,%10.4f", ins.latitude, ins.longitude, ins.height);
			fprintf(f_mi_ins_csv, ",%14.9f,%14.9f,%10.4f", ins.heading*D2R, ins.roll*D2R, ins.pitch*D2R);
			fprintf(f_mi_ins_csv, ",%10.4f,%10.4f,%10.4f", ins.east_velocity, ins.north_velocity, ins.up_velocity);
			fprintf(f_mi_ins_csv, ",%14.10f,%14.10f,%14.10f", imu.accel_x, imu.accel_y, imu.accel_z);
			fprintf(f_mi_ins_csv, ",%14.10f,%14.10f,%14.10f", imu.gyro_x*D2R, imu.gyro_y*D2R, imu.gyro_z*D2R);
			fprintf(f_mi_ins_csv, ",%8.3f,%8.3f,%8.3f", ins.latitude_std, ins.longitude_std, ins.height_std);
			fprintf(f_mi_ins_csv, ",%8.3f,%8.3f,%8.3f", ins.east_velocity_std, ins.north_velocity_std, ins.up_velocity_std);
			fprintf(f_mi_ins_csv, ",%14.9f,%14.9f,%14.9f", ins.heading_std*D2R, ins.roll_std*D2R, ins.pitch_std*D2R);
			fprintf(f_mi_ins_csv, ",%10.3f \n", height_msl);	/*height_msl*/
		}
#endif // MI_OUTPUT_FILE
	}

	void Ins401_decoder::output_misa_sol() {
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s),flag()"
			",RVB1(),RVB2(),RVB3()"
			",CVB1(),CVB2(),CVB3()\n";
		create_file(f_misa_csv, "misa.csv", title.c_str(), show_format_time);
		if (show_format_time) {
			fprintf(f_misa_csv,"%s,", week_2_time_str(misa.gps_week, misa.gps_millisecs));
		}
		fprintf(f_misa_csv
			,"%d,%d,%d"
			",%f,%f,%f"
			",%f,%f,%f\n"
			, misa.gps_week, misa.gps_millisecs, misa.flag
			, misa.RVB[0], misa.RVB[1], misa.RVB[2]
			, misa.CVB[0], misa.CVB[1], misa.CVB[2]);
#ifdef OUTPUT_INNER_FILE
		create_file(f_misa_txt, "misa.txt", NULL);
		fprintf(f_misa_txt
			, "%d,%d,%d"
			",%f,%f,%f"
			",%f,%f,%f\n"
			, misa.gps_week, misa.gps_millisecs, misa.flag
			, misa.RVB[0], misa.RVB[1], misa.RVB[2]
			, misa.CVB[0], misa.CVB[1], misa.CVB[2]);
		create_file(f_process, "process", NULL);
		fprintf(f_process
			, "$GPMISA,%d,%d,%d"
			",%f,%f,%f"
			",%f,%f,%f\n"
			, misa.gps_week, misa.gps_millisecs, misa.flag
			, misa.RVB[0], misa.RVB[1], misa.RVB[2]
			, misa.CVB[0], misa.CVB[1], misa.CVB[2]);
#endif
	}

	void Ins401_decoder::output_odo_raw()
	{
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s),mode(),speed(m/s),fwd(),wheel_tick()\n";
		create_file(f_odo_csv, "odo.csv", title.c_str(), show_format_time);
		if (show_format_time) {
			fprintf(f_odo_csv, "%s,", week_2_time_str(odo.GPS_Week, odo.GPS_TimeOfWeek));
		}
		fprintf(f_odo_csv, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n"
			, odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);
#ifdef OUTPUT_INNER_FILE
		create_file(f_odo_txt, "odo.txt", NULL);
		fprintf(f_odo_txt, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n"
			, odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);

		create_file(f_process, "process", NULL);
		fprintf(f_process, "$GPODO,%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n"
			, odo.GPS_Week, (double)odo.GPS_TimeOfWeek / 1000.0, odo.mode, odo.speed, odo.fwd, odo.wheel_tick);
#endif
	}

	void Ins401_decoder::output_dm_raw() {

		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",master_fail, hw_err, sw_err, config_err, calib_err, accel_degradation, gyro_degradation"
			",forced_restart, crc_err, tx_overflow_err, pps_status, gnss_data_status, gnss_signal_status"
			",power, MCU_status, temperature_under_mcu_flag, temperature_under_sta_flag, temperature_under_imu_flag"
			",temperature_over_mcu_flag, temperature_over_sta_flag, temperature_over_imu_flag"
			",IMU Temperature(),MCU Temperature(),STA9100 Temperature()\n";
		create_file(f_dm_csv, "dm.csv", title.c_str(), show_format_time);
		if (show_format_time) {
			fprintf(f_dm_csv, "%s,", week_2_time_str(dm.gps_week, dm.gps_millisecs));
		}
		fprintf(f_dm_csv, "%d,%11.3f"
			",%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d,%2d"
			",%5.1f,%5.1f,%5.1f\n"
			,dm.gps_week, (double)dm.gps_millisecs / 1000.0,
			dm.status_bit.master_fail, dm.status_bit.hw_err, dm.status_bit.sw_err, dm.status_bit.config_err, dm.status_bit.calib_err, dm.status_bit.accel_degradation, dm.status_bit.gyro_degradation,
			dm.status_bit.forced_restart, dm.status_bit.crc_err, dm.status_bit.tx_overflow_err, dm.status_bit.pps_status, dm.status_bit.gnss_data_status, dm.status_bit.gnss_signal_status,
			dm.status_bit.power, dm.status_bit.MCU_status, dm.status_bit.temperature_under_mcu_flag, dm.status_bit.temperature_under_sta_flag, dm.status_bit.temperature_under_imu_flag,
			dm.status_bit.temperature_over_mcu_flag, dm.status_bit.temperature_over_sta_flag, dm.status_bit.temperature_over_imu_flag,
			dm.IMU_Unit_temperature, dm.MCU_temperature, dm.STA9100_temperature);
	}

	void Ins401_decoder::output_gnss_integ()
	{
		if (last_gnss_integ_millisecs == (uint32_t)gnss_integ.gps_millisecs)
			return;
		FILE* f_gnss_integ_csv = NULL;
		std::string gnss_integ_name = "gnss_integrity.csv";
		FilesMap::iterator it = output_file_map.find(gnss_integ_name);
		if (it == output_file_map.end()) {
			std::string gnss_integ_title =
				"week,timeOfWeek(s),spp_fail_rate,rtk_fail_rate"
				",spp_hor_pos_pl(m),spp_ver_pos_pl(m),spp_hor_vel_pl(m/s),spp_ver_vel_pl(m/s)"
				",rtk_hor_pos_pl(m),rtk_ver_pos_pl(m),rtk_hor_vel_pl(m/s),rtk_ver_vel_pl(m/s),rtk_heading_pl(deg)"
				",spp_hor_pos_al(m),spp_ver_pos_al(m),spp_hor_vel_al(m/s),spp_ver_vel_al(m/s)"
				",rtk_hor_pos_al(m),rtk_ver_pos_al(m),rtk_hor_vel_al(m/s),rtk_ver_vel_al(m/s),rtk_heading_pl(deg)"
				",spp_hor_pos_stat,spp_ver_pos_stat,spp_hor_vel_stat,spp_ver_vel_stat"
				",rtk_hor_pos_stat,rtk_ver_pos_stat,rtk_hor_vel_stat,rtk_ver_vel_stat,rtk_heading_stat\n";
			create_file(f_gnss_integ_csv, gnss_integ_name.c_str(), gnss_integ_title.c_str());
			output_file_map[gnss_integ_name] = f_gnss_integ_csv;
		}
		f_gnss_integ_csv = output_file_map[gnss_integ_name];
		if (f_gnss_integ_csv) {
			fprintf(f_gnss_integ_csv, "%4d,%11.4f,%8.3f,%8.3f", gnss_integ.week, gnss_integ.gps_millisecs / 1000, (float)gnss_integ.spp_fail_rate / 1.0e-10, (float)gnss_integ.rtk_fail_rate / 1.0e-10);
			fprintf(f_gnss_integ_csv, ",%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.spp_hor_pos_pl / 100, (float)gnss_integ.spp_ver_pos_pl / 100, (float)gnss_integ.spp_hor_vel_pl / 100, (float)gnss_integ.spp_ver_vel_pl / 100);
			fprintf(f_gnss_integ_csv, ",%8.3f,%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.rtk_hor_pos_pl / 100, (float)gnss_integ.rtk_ver_pos_pl / 100, (float)gnss_integ.rtk_hor_vel_pl / 100, (float)gnss_integ.rtk_ver_vel_pl / 100, (float)gnss_integ.rtk_heading_pl / 100);
			fprintf(f_gnss_integ_csv, ",%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.spp_hor_pos_al / 100, (float)gnss_integ.spp_ver_pos_al / 100, (float)gnss_integ.spp_hor_vel_al / 100, (float)gnss_integ.spp_ver_vel_al / 100);
			fprintf(f_gnss_integ_csv, ",%8.3f,%8.3f,%8.3f,%8.3f,%8.3f", (float)gnss_integ.rtk_hor_pos_al / 100, (float)gnss_integ.rtk_ver_pos_al / 100, (float)gnss_integ.rtk_hor_vel_al / 100, (float)gnss_integ.rtk_ver_vel_al / 100, (float)gnss_integ.rtk_heading_al / 100);
			fprintf(f_gnss_integ_csv, ",%2d,%2d,%2d,%2d", gnss_integ.status_bit.spp_hor_pos_s, gnss_integ.status_bit.spp_ver_pos_s, gnss_integ.status_bit.spp_hor_vel_s, gnss_integ.status_bit.spp_ver_vel_s);
			fprintf(f_gnss_integ_csv, ",%2d,%2d,%2d,%2d,%2d", gnss_integ.status_bit.rtk_hor_pos_s, gnss_integ.status_bit.rtk_ver_pos_s, gnss_integ.status_bit.rtk_hor_vel_s, gnss_integ.status_bit.rtk_ver_vel_s, gnss_integ.status_bit.rtk_heading_s);
			fprintf(f_gnss_integ_csv, "\n");
		}
	}

	void Ins401_decoder::output_movbs_sol()
	{
		std::string title = 
			"GPS_Week(),GPS_TimeOfWeek(s),masterFlag(),slaveFlag()"
			",masterType(),slaveType(),masterSatView()"
			",masterSatSol(),slaveSatView(),slaveSatSol()"
			",HDOP(),VDOP(),TDOP(),PDOP()"
			",solAge(),masterLon(deg),masterLat(deg), masterHg(m)"
			",slaveLon(deg),slaveLat(deg),slaveHg(m)"
			",roll(deg),pitch(deg),heading(deg)"
			",geoSep(m),masterVelN(m/s),masterVelE(m/s),masterVelU(m/s)"
			",slaveVelN(m/s),slaveVelE(m/s),slaveVelU(m/s)"
            ",lonStd(),latStd(),hgStd()"
            ",velNStd(),velEStd(),velUStd()"
            ",horPosPl(),verPosPl(),horVelPl(),verVelPl()"
            ",posPlStatus(),velPlStatus(),fwVer()"
			",contient()\n";
		create_file(f_movbs_csv, "movbs.csv", title.c_str(), show_format_time);
		if (show_format_time) {
			fprintf(f_movbs_csv, "%s,", week_2_time_str(movbs.week, movbs.tow));
		}
		fprintf(f_movbs_csv, "%d,%11.4f,", movbs.week, (double)movbs.tow);
		fprintf(f_movbs_csv, "%d,%d,%d,%d,", movbs.masterFlag, movbs.slaveFlag, movbs.masterType, movbs.slaveType);
		fprintf(f_movbs_csv, "%d,%d,%d,%d,", movbs.masterSatView, movbs.masterSatSol, movbs.slaveSatView, movbs.slaveSatSol);
		fprintf(f_movbs_csv, "%10.4f,%10.4f,%10.4f,%10.4f", movbs.HDOP, movbs.VDOP, movbs.TDOP, movbs.PDOP);
		fprintf(f_movbs_csv, "%4.2f,", movbs.solAge);
		fprintf(f_movbs_csv, "%14.9f,%14.9f,%14.9f,", movbs.masterLon, movbs.masterLat, movbs.masterHg);
		fprintf(f_movbs_csv, "%14.9f,%14.9f,%14.9f,", movbs.slaveLon, movbs.slaveLat, movbs.slaveHg);
		fprintf(f_movbs_csv, "%8.3f,%8.3f,%8.3f,,%8.3f", movbs.roll, movbs.pitch, movbs.heading, movbs.geoSep);
		fprintf(f_movbs_csv, "%8.3f,%8.3f,,%8.3f", movbs.masterVelN, movbs.masterVelE, movbs.masterVelU);
		fprintf(f_movbs_csv, "%8.3f,%8.3f,%8.3f,", movbs.slaveVelN, movbs.slaveVelE, movbs.slaveVelU);
		fprintf(f_movbs_csv, "%8.3f,%8.3f,%8.3f,", movbs.lonStd, movbs.latStd, movbs.hgStd);
		fprintf(f_movbs_csv, "%8.3f,%8.3f,%8.3f,", movbs.velNStd, movbs.velEStd, movbs.velUStd);
		fprintf(f_movbs_csv, "%8.3f,%8.3f,%8.3f,,%8.3f", movbs.horPosPl, movbs.verPosPl, movbs.horVelPl, movbs.verVelPl);
		fprintf(f_movbs_csv, "%d,%d,%d\n", movbs.posPlStatus, movbs.velPlStatus, movbs.fwVer);

#ifdef OUTPUT_INNER_FILE
#if 1
			//txt
			create_file(f_ins_txt, "movbs.txt", NULL);
			fprintf(f_ins_txt, "$GPMOVBS,%d,%11.4f,%d,%d,%d,%d,%10.4f,%10.4f,%10.4f\n",
				movbs.week, movbs.tow, movbs.masterFlag, movbs.slaveFlag, movbs.masterType,
				movbs.slaveType, movbs.roll, movbs.pitch, movbs.heading);
			
            //process
			create_file(f_process, "process", NULL);
			fprintf(f_process, "$GPMOVBS,%d,%11.4f,%d,%d,%d,%d,%10.4f,%10.4f,%10.4f\n",
				movbs.week, movbs.tow, movbs.masterFlag, movbs.slaveFlag, movbs.masterType,
				movbs.slaveType, movbs.roll, movbs.pitch, movbs.heading);
#endif
#endif
	}

	void Ins401_decoder::output_gnss_sol_and_integ()
	{
		if (gnss.gps_week == 0 || gnss_integ.week == 0)
			return;
		if (last_gnss_integ_millisecs == (uint32_t)gnss_integ.gps_millisecs)
			return;
		if (gnss.gps_millisecs != (uint32_t)gnss_integ.gps_millisecs)
			return;
		FILE* f_gnss_sol_integ_csv = NULL;
		std::string gnss_sol_integ_name = "gnss_and_integrity.csv";
		FilesMap::iterator it = output_file_map.find(gnss_sol_integ_name);
		if (it == output_file_map.end()) {
			std::string gnss_and_integ_title =
				"GPS_Week(),GPS_TimeOfWeek(s),position_type()"
				",latitude(deg),longitude(deg),height(m)"
				",latitude_standard_deviation(m),longitude_standard_deviation(m),height_standard_deviation(m)"
				",number_of_satellites(),number_of_satellites_in_solution()"
				",hdop(),diffage(s)"
				",north_vel(m/s),east_vel(m/s),up_vel(m/s)"
				",north_vel_standard_deviation(m/s),east_vel_standard_deviation(m/s),up_vel_standard_deviation(m/s)"
				",rtk_hor_pos_pl(m),rtk_ver_pos_pl(m),rtk_hor_pos_stat,rtk_ver_pos_stat"
				",rtk_hor_vel_pl(m/s),rtk_ver_vel_pl(m/s),rtk_hor_vel_stat,rtk_ver_vel_stat\n";
			create_file(f_gnss_sol_integ_csv, gnss_sol_integ_name.c_str(), gnss_and_integ_title.c_str());
			output_file_map[gnss_sol_integ_name] = f_gnss_sol_integ_csv;
		}
		f_gnss_sol_integ_csv = output_file_map[gnss_sol_integ_name];
		if (f_gnss_sol_integ_csv) {
			fprintf(f_gnss_sol_integ_csv, "%d,%11.4f,%3d", gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.position_type);
			fprintf(f_gnss_sol_integ_csv, ",%14.9f,%14.9f,%10.4f", gnss.latitude, gnss.longitude, gnss.height);
			fprintf(f_gnss_sol_integ_csv, ",%10.4f,%10.4f,%10.4f", gnss.latitude_std, gnss.longitude_std, gnss.height_std);
			fprintf(f_gnss_sol_integ_csv, ",%3d,%3d", gnss.numberOfSVs, gnss.numberOfSVs_in_solution);
			fprintf(f_gnss_sol_integ_csv, ",%5.1f,%5.1f", gnss.hdop, gnss.diffage);
			fprintf(f_gnss_sol_integ_csv, ",%10.4f,%10.4f,%10.4f", gnss.north_vel, gnss.east_vel, gnss.up_vel);
			fprintf(f_gnss_sol_integ_csv, ",%10.4f,%10.4f,%10.4f", gnss.north_vel_std, gnss.east_vel_std, gnss.up_vel_std);
			fprintf(f_gnss_sol_integ_csv, ",%8.3f,%8.3f,%2d,%2d", (float)gnss_integ.rtk_hor_pos_pl / 100, (float)gnss_integ.rtk_ver_pos_pl / 100, gnss_integ.status_bit.rtk_hor_pos_s, gnss_integ.status_bit.rtk_ver_pos_s);
			fprintf(f_gnss_sol_integ_csv, ",%8.3f,%8.3f,%2d,%2d", (float)gnss_integ.rtk_hor_vel_pl / 100, (float)gnss_integ.rtk_ver_vel_pl / 100, gnss_integ.status_bit.rtk_hor_vel_s, gnss_integ.status_bit.rtk_ver_vel_s);
			fprintf(f_gnss_sol_integ_csv, "\n");
		}
	}

	void Ins401_decoder::output_check()
	{
		FILE* f_check_csv = NULL;
		std::string check_name = "check.csv";
		FilesMap::iterator it = output_file_map.find(check_name);
		if (it == output_file_map.end()) {
			std::string check_title = "";
			create_file(f_check_csv, check_name.c_str(), check_title.c_str());
			output_file_map[check_name] = f_check_csv;
		}
		f_check_csv = output_file_map[check_name];
		if (f_check_csv) {

		}
	}

	void Ins401_decoder::output_rtk_debug1()
	{
		FILE* f_rtk_debug1_csv = NULL;
		std::string rtk_debug1_name = "rtk_debug1.csv";
		FilesMap::iterator it = output_file_map.find(rtk_debug1_name);
		if (it == output_file_map.end()) {
			std::string check_title = "";
			create_file(f_rtk_debug1_csv, rtk_debug1_name.c_str(), check_title.c_str());
			output_file_map[rtk_debug1_name] = f_rtk_debug1_csv;
		}
		f_rtk_debug1_csv = output_file_map[rtk_debug1_name];
		if (f_rtk_debug1_csv) {
			fprintf(f_rtk_debug1_csv, "%d,%11.4f,%d\n", rtk_debug1.gps_week, (double)rtk_debug1.gps_millisecs / 1000.0, rtk_debug1.ins_aid);
		}
	}

	void Ins401_decoder::output_system_fault_detection()
	{
		FILE* f_fd = NULL;
		std::string fd_name = "fd.csv";
		FilesMap::iterator it = output_file_map.find(fd_name);
		if (it == output_file_map.end()) {
			std::string check_title = "";
			create_file(f_fd, fd_name.c_str(), check_title.c_str());
			output_file_map[fd_name] = f_fd;
		}
		f_fd = output_file_map[fd_name];
		if (f_fd) {
			fprintf(f_fd, "%d,%d,%d,%d\n",
				system_fault_detection.gnss_err_out_pin,
				system_fault_detection.gnss_rf_err_pin,
				system_fault_detection.gnss_ant_err_pin,
				system_fault_detection.gnss_handshake_flag);
		}
	}

	void Ins401_decoder::output_rover_rtcm()
	{
		create_file(f_rover_rtcm, "_rover.rtcm", NULL);
		fwrite(raw.buff + 6, 1, raw.length, f_rover_rtcm);
	}

	void Ins401_decoder::parse_packet_payload()
	{
		uint8_t* payload = raw.buff + 6;
		switch (raw.packet_type) {
		case em_RAW_IMU:
		{
			size_t packet_size = sizeof(raw_imu_t);
			if (raw.length == packet_size) {
				memcpy(&imu, payload, packet_size);
				output_imu_raw();
				MI_output_imu_raw();
#ifdef OUTPUT_INNER_FILE
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
				//MI_output_gnss_sol();
				output_gnss_sol_and_integ();
			}
		}break;
		case em_INS_SOL:
		{
			size_t packet_size_20211207 = sizeof(ins_sol_t_20211207);
			size_t packet_size = sizeof(ins_sol_t);
			if (raw.length == packet_size || raw.length == packet_size_20211207) {
				memcpy(&ins, payload, raw.length);
				output_ins_sol();
				MI_output_ins_sol();
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
		case PowerUpDR_MES:
		{
			size_t packet_size = sizeof(SaveMsg);
			int ret = 0;
			for (uint32_t i = 0; i < raw.length; i++)
			{
				ret = input_ins_save_data(raw.buff[i]);
			}
		}
		break;
		case em_CHECK:
		{
			size_t packet_size = sizeof(stPacketCheck);
			if (raw.length == packet_size){
				memcpy(&packetcheck, payload, packet_size);
			}
		}
		break;
		case em_GNSS_SOL_INTEGEITY:
		{
			size_t packet_size = sizeof(gnss_integ_t);
			if (raw.length == packet_size) {
				memcpy(&gnss_integ, payload, packet_size);
				output_gnss_integ();
				output_gnss_sol_and_integ();
				last_gnss_integ_millisecs = (uint32_t)gnss_integ.gps_millisecs;
			}
		}
		break;
		case em_RTK_DEBUG1:
		{
#ifdef OUTPUT_INNER_FILE
			size_t packet_size = sizeof(binary_rtk_debug1_t);
			if (raw.length == packet_size) {
				memcpy(&rtk_debug1, payload, packet_size);		
				output_rtk_debug1();
			}
#endif
		}
		break;
		case em_PACKAGE_FD:
		{
			size_t packet_size = sizeof(system_fault_detection_t);
			if (raw.length == packet_size) {
				memcpy(&system_fault_detection, payload, packet_size);
				output_system_fault_detection();
			}
		}
		break;
        case em_MOVBS_SOL:
		{
			size_t packet_size = sizeof(movbs_sol_t);
			if (raw.length == packet_size) {
				memcpy(&movbs, payload, packet_size);
				output_movbs_sol();
			}
		}
        break;

		default:
			break;
		};
	}

	void Ins401_decoder::save_imu_bin()
	{
		create_file(f_imu_bin, "imu.bin", NULL);
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

	void Ins401_decoder::parse_gga() {
		if (m_MI_file_switch == false) return;
		if (strncmp((char*)raw.nmea, nmea_type(1), NMEA_HEADER_LEN) == 0) {
			char *p = strtok((char*)raw.nmea, ",");
			int pos = 0;
			while (p) {
				if (pos == 9) {
					height_msl = atof(p);
					MI_output_gnss_sol();
					break;
				}
				p = strtok(NULL, ",");
				pos++;
			}
		}
	}

	int8_t Ins401_decoder::parse_nmea(uint8_t data)
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
				memcpy(NMEA, raw.nmea, NMEA_HEADER_LEN);
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
				create_file(f_nmea, "nmea.txt", NULL);
				fprintf(f_nmea, (char*)raw.nmea);
#ifdef MI_OUTPUT_FILE
				parse_gga();
#endif //MI_OUTPUT_FILE
				return 2;
			}
		}
		return 0;
	}

	void Ins401_decoder::set_base_file_name(char * file_name)
	{
		strcpy(base_file_name, file_name);
	}

	void Ins401_decoder::set_show_format_time(bool show)
	{
		show_format_time = show;
	}

	void Ins401_decoder::set_MI_file_switch(bool write)
	{
		m_MI_file_switch = write;
	}

	int Ins401_decoder::input_data(uint8_t data)
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
			if (raw.nbyte == 6) {
				memcpy(&raw.length, &raw.buff[2], sizeof(uint32_t));
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
					//create_file(f_log, ".log", NULL);
					//fprintf(f_log, "crc failed read type = %04X, len = %d, crc = %d, calc crc = %d\n", raw.packet_type, raw.length, packet_crc, crc);
				}
				raw.flag = 0;
				raw.nbyte = 0;
				raw.length = 0;
			}
		}
		return ret;
	}

	void Ins401_decoder::finish()
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

	int Ins401_decoder::get_current_type()
	{
		return raw.packet_type;
	}

	gnss_sol_t * Ins401_decoder::get_gnss_sol()
	{
		return &gnss;
	}

	ins_sol_t * Ins401_decoder::get_ins_sol()
	{
		return &ins;
	}

	raw_imu_t * Ins401_decoder::get_imu_raw()
	{
		return &imu;
	}
};
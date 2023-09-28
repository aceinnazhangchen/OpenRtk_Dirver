#include "rtk350la.h"
#include "common.h"
#include "rtklib_core.h"
#include <string.h>
#include <math.h>

namespace RTK350LA_Tool {

	RTK350LA_decoder::RTK350LA_decoder()
		: m_isOutputFile(true)
	{
		all_type_pack_num.clear();
		output_file_map.clear();
		init();
	}


	RTK350LA_decoder::~RTK350LA_decoder()
	{
		close_all_files();
	}


	void RTK350LA_decoder::close_all_files()
	{
		FilesMap::iterator it;
		for (it = output_file_map.begin(); it != output_file_map.end(); it++) {
			if (it->second) fclose(it->second); it->second = NULL;
		}
		output_file_map.clear();
	}

	void RTK350LA_decoder::create_file(FILE *& file, const char * suffix, const char * title, bool format_time = false)
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

	FILE * RTK350LA_decoder::get_file(std::string suffix, std::string title = "", bool format_time = false)
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

	void RTK350LA_decoder::init()
	{
		is_pruned = false;
		m_isOutputFile = true;
		data_version = 0;
		pack_num = 0;
		crc_right_num = 0;
		crc_error_num = 0;
		show_format_time = false;
		memset(base_file_name, 0, 256);
		memset(output_msg, 0, MAX_OUTPUT_MSG_LEN);
		memset(&raw, 0, sizeof(raw));

		memset(&pak_S2, 0, sizeof(pak_S2));
        memset(&pak_SI, 0, sizeof(pak_SI));

		memset(&pak_gN, 0, sizeof(pak_gN));
		memset(&pak_iN, 0, sizeof(pak_iN));
		memset(&pak_o1, 0, sizeof(pak_o1));
		memset(&pak_rr, 0, sizeof(pak_rr));
		memset(&pak_pvt, 0, sizeof(pak_pvt));
        memset(&pak_bias, 0, sizeof(pak_bias));
        memset(&pak_sT, 0, sizeof(pak_sT));
        memset(&pak_iS, 0, sizeof(pak_iS));
		memset(&gnss_kml, 0, sizeof(gnss_kml));
		memset(&ins_kml, 0, sizeof(ins_kml));

		all_type_pack_num[em_gN] = 0;
		all_type_pack_num[em_S2] = 0;
        all_type_pack_num[em_SI] = 0;
		all_type_pack_num[em_iN] = 0;
		all_type_pack_num[em_o1] = 0;
		all_type_pack_num[em_iS] = 0;
		all_type_pack_num[em_pv] = 0;
		all_type_pack_num[em_rR] = 0;        
		all_type_pack_num[em_iB] = 0;
        all_type_pack_num[em_sT] = 0;   

		all_type_file_output[em_gN] = 1;
		all_type_file_output[em_S2] = 1;
        all_type_file_output[em_SI] = 1;   
		all_type_file_output[em_iN] = 1;
		all_type_file_output[em_o1] = 1;
		all_type_file_output[em_iS] = 1;
		all_type_file_output[em_pv] = 1;
		all_type_file_output[em_rR] = 1;
		all_type_file_output[em_iB] = 1;
        all_type_file_output[em_sT] = 1;
	}

	void RTK350LA_decoder::set_output_file(bool output)
	{
		m_isOutputFile = output;
	}

	void RTK350LA_decoder::set_base_file_name(char * file_name)
	{
		strcpy(base_file_name, file_name);
	}

	void RTK350LA_decoder::set_pruned(bool pruned)
	{
		is_pruned = pruned;
	}

	void RTK350LA_decoder::append_gnss_kml() {
		gnss_kml.gps_week = pak_gN.week;
		gnss_kml.gps_secs = pak_gN.timeOfWeek / 1000.0;
		gnss_kml.position_type = pak_gN.positionMode;
		gnss_kml.latitude = (double)pak_gN.latitude*180.0 / MAX_INT ;
		gnss_kml.longitude = (double)pak_gN.longitude*180.0 / MAX_INT ;
		gnss_kml.height = pak_gN.height;
		gnss_kml.north_vel = (float)pak_gN.north_vel / 100.0f;
		gnss_kml.east_vel = (float)pak_gN.east_vel / 100.0f;
		gnss_kml.up_vel = (float)pak_gN.up_vel / 100.0f;
		Kml_Generator::Instance()->append_gnss(gnss_kml);
	}

	void RTK350LA_decoder::append_ins_kml() {
		ins_kml.gps_week = pak_iN.week;
		ins_kml.gps_secs = pak_iN.timeOfWeek / 1000.0;
		ins_kml.ins_status = pak_iN.insStatus;
		ins_kml.ins_position_type = pak_iN.insPositionType;
		ins_kml.latitude = (double)pak_iN.latitude*180.0 / MAX_INT;
		ins_kml.longitude = (double)pak_iN.longitude*180.0 / MAX_INT;
		ins_kml.height = pak_iN.height;
		ins_kml.north_velocity = (float)pak_iN.north_vel / 100.0f;
		ins_kml.east_velocity = (float)pak_iN.east_vel / 100.0f;
		ins_kml.up_velocity = (float)pak_iN.up_vel / 100.0f;
		ins_kml.roll = (float)pak_iN.roll / 100.0f;
		ins_kml.pitch = (float)pak_iN.pitch / 100.0f;
		ins_kml.heading = (float)pak_iN.heading / 100.0f;
		Kml_Generator::Instance()->append_ins(ins_kml);
	}

	void RTK350LA_decoder::output_S2() {
		//csv
		std::string name_csv = "S2.csv";
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2)"
			",x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n";
		FILE* fp_csv = get_file(name_csv, title);
		sprintf(output_msg,
			"%d,%11.4f"
			",%14.10f,%14.10f,%14.10f"
			",%14.10f,%14.10f,%14.10f\n",
			pak_S2.week, pak_S2.timeOfWeek / 1000.0,
			pak_S2.accel_g[0], pak_S2.accel_g[1], pak_S2.accel_g[2],
			pak_S2.rate_dps[0], pak_S2.rate_dps[1], pak_S2.rate_dps[2]);
		if (fp_csv) fprintf(fp_csv, output_msg);
		FILE* fp_txt = get_file("imu.txt");
		if (fp_txt) fprintf(fp_txt, output_msg);
		FILE* fp_process = get_file("process");
		if (fp_process) fprintf(fp_process, "$GPIMU,%s", output_msg);

	}

	void RTK350LA_decoder::output_SI() {
		//csv
		std::string name_csv = "SI.csv";
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2)"
			",x_rate(deg/s),y_rate(deg/s),z_rate(deg/s)\n";
		FILE* fp_csv = get_file(name_csv, title);
		sprintf(output_msg,
			"%d,%11.4f"
			",%14.10f,%14.10f,%14.10f"
			",%14.10f,%14.10f,%14.10f\n",
			pak_SI.week, pak_SI.timeOfWeek / 1000.0,
			pak_SI.accel_g[0], pak_SI.accel_g[1], pak_SI.accel_g[2],
			pak_SI.rate_dps[0], pak_SI.rate_dps[1], pak_SI.rate_dps[2]);
		if (fp_csv) fprintf(fp_csv, output_msg);
	}



	void RTK350LA_decoder::output_gN() {
		float north_vel = (float)pak_gN.north_vel / 100.0f;
		float east_vel = (float)pak_gN.east_vel / 100.0f;
		float up_vel = (float)pak_gN.up_vel / 100.0f;
		float latitude_std = (float)pak_gN.latitude_std / 1000.0f;
		float longitude_std = (float)pak_gN.longitude_std / 1000.0f;
		float height_std = (float)pak_gN.height_std / 1000.0f;
        float north_vel_std = (float)pak_gN.north_vel_std / 1000.0f;
        float east_vel_std = (float)pak_gN.east_vel_std / 1000.0f;
        float up_vel_std = (float)pak_gN.up_vel_std / 1000.0f;

		double horizontal_speed = sqrt(north_vel * north_vel + east_vel * east_vel);
		double track_over_ground = atan2(east_vel, north_vel) * R2D;
		//csv
		std::string title =
			"GPS_Week(),GPS_TimeofWeek(s),positionMode()"
			",latitude(deg),longitude(deg),height(m)"
			",numberOfSVs(),hdop(),vdop(),tdop(),diffage()"
			",velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s)"
			",latitude_std(m),longitude_std(m),height_std(m)"
			",north_vel_std(m/s),east_vel_std(m/s),up_vel_std(m/s)\n";
		FILE* f_gN = get_file("gN.csv",title);
		sprintf(output_msg,
			"%d,%11.4f,%3d"
			",%14.9f,%14.9f,%10.4f"
			",%3d,%5.1f,%5.1f,%5.1f,%5.1f"
			",%10.4f,%10.4f,%10.4f"
			",%10.4f,%10.4f,%10.4f"
			",%10.4f,%10.4f,%10.4f\n"
			, pak_gN.week, pak_gN.timeOfWeek / 1000.0, pak_gN.positionMode
			, (double)pak_gN.latitude*180.0 / MAX_INT, (double)pak_gN.longitude*180.0 / MAX_INT, pak_gN.height
			, pak_gN.numberOfSVs, pak_gN.hdop, pak_gN.vdop, pak_gN.tdop, (float)pak_gN.diffage
			, north_vel, east_vel, up_vel
			, latitude_std, longitude_std, height_std
			, north_vel_std, east_vel_std, up_vel_std
			);
		if (f_gN) fprintf(f_gN, output_msg);

		//process $GPGNSS
		FILE* f_process = get_file("process");
		sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%3d\n"
			, pak_gN.week, pak_gN.timeOfWeek / 1000.0
			, (double)pak_gN.latitude*180.0 / MAX_INT, (double)pak_gN.longitude*180.0 / MAX_INT
			, pak_gN.height, latitude_std, longitude_std, height_std
			, pak_gN.positionMode, pak_gN.diffage);
		if (f_process) fprintf(f_process, "$GPGNSS,%s", output_msg);
		//process $GPVEL
		sprintf(output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f\n", pak_gN.week, pak_gN.timeOfWeek / 1000.0, horizontal_speed, track_over_ground, up_vel);
		if (f_process) fprintf(f_process, "$GPVEL,%s", output_msg);
		//process $GPVNED
		sprintf(output_msg, "%d,%11.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", pak_gN.week, pak_gN.timeOfWeek / 1000.0, north_vel, east_vel, -up_vel,
			north_vel_std, east_vel_std, up_vel_std);
		if (f_process) fprintf(f_process, "$GPVNED,%s", output_msg);
		if (is_pruned)return;
		//txt
		FILE* f_gnssposvel = get_file("gnssposvel.txt");
		sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%3d,%10.4f,%10.4f,%10.4f,%10.4f\n",
			pak_gN.week, pak_gN.timeOfWeek / 1000.0, pak_gN.latitude*180.0 / MAX_INT, pak_gN.longitude*180.0 / MAX_INT, pak_gN.height,
			latitude_std, longitude_std, height_std, pak_gN.positionMode, north_vel, east_vel, up_vel, track_over_ground);
		if (f_gnssposvel) fprintf(f_gnssposvel, output_msg);
		//kml
		append_gnss_kml();
		//time
		FILE* f_gnss_time_txt = get_file("gnss_process_time.csv");
		if (f_gnss_time_txt) fprintf(f_gnss_time_txt, "%d,%11.4f,%6.4f\n", pak_gN.week, pak_gN.timeOfWeek / 1000.0, (pak_S2.timeOfWeek - pak_gN.timeOfWeek));
		//last_GPS_TimeOfWeek = inceptio_pak_gN.GPS_TimeOfWeek;
	}

	void RTK350LA_decoder::output_iN() {
		uint32_t GPS_TimeOfWeek = pak_iN.timeOfWeek;
		if (GPS_TimeOfWeek % 100 == 0) {
			//txt
			std::string title_txt =
				"GPS_Week(),GPS_TimeOfWeek(s)"
				",latitude(deg),longitude(deg),height(m)"
				",north_velocity(m/s),east_velocity(m/s),up_velocity(m/s)"
				",roll(deg),pitch(deg),heading(deg)"
				",ins_position_type(),ins_status()\n";
			FILE* f_ins = get_file("ins.txt", title_txt);
			sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d,%3d\n", pak_iN.week, pak_iN.timeOfWeek / 1000.0,
				(double)pak_iN.latitude*180.0 / MAX_INT, (double)pak_iN.longitude*180.0 / MAX_INT, pak_iN.height,
				(float)pak_iN.north_vel / 100.0, (float)pak_iN.east_vel / 100.0, (float)pak_iN.up_vel / 100.0,
				(float)pak_iN.roll / 100.0, (float)pak_iN.pitch / 100.0, (float)pak_iN.heading / 100.0, pak_iN.insPositionType, pak_iN.insStatus);
			if (f_ins) fprintf(f_ins, output_msg);
			//process
			FILE* f_process = get_file("process");
			sprintf(output_msg, "%d,%11.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%3d\n", pak_iN.week, pak_iN.timeOfWeek / 1000.0,
				(double)pak_iN.latitude*180.0 / MAX_INT, (double)pak_iN.longitude*180.0 / MAX_INT, pak_iN.height,
				(float)pak_iN.north_vel / 100.0, (float)pak_iN.east_vel / 100.0, (float)pak_iN.up_vel / 100.0,
				(float)pak_iN.roll / 100.0, (float)pak_iN.pitch / 100.0, (float)pak_iN.heading / 100.0, pak_iN.insPositionType);
			if (f_process) fprintf(f_process, "$GPINS,%s", output_msg);
		}
		if (is_pruned)return;
		//csv
		std::string title =
			"GPS_Week(),GPS_TimeofWeek(s)"
			",insStatus(),insPositionType()"
			",latitude(deg),longitude(deg),height(m)"
			",velocityNorth(m/s),velocityEast(m/s),velocityUp(m/s)"
			",roll(deg),pitch(deg),heading(deg)\n";
		FILE* f_iN = get_file("iN.csv", title);
		sprintf(output_msg, "%d,%11.4f,%3d,%3d,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f\n", pak_iN.week, pak_iN.timeOfWeek / 1000.0,
			pak_iN.insStatus, pak_iN.insPositionType,
			(double)pak_iN.latitude*180.0 / MAX_INT, (double)pak_iN.longitude*180.0 / MAX_INT, pak_iN.height,
			(float)pak_iN.north_vel / 100.0, (float)pak_iN.east_vel / 100.0, (float)pak_iN.up_vel / 100.0,
			(float)pak_iN.roll / 100.0, (float)pak_iN.pitch / 100.0, (float)pak_iN.heading / 100.0);
		if (f_iN) fprintf(f_iN, output_msg);
		//kml
		append_ins_kml();
	}

	void RTK350LA_decoder::output_o1() {
		//csv
		std::string title = 
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",mode(),speed(m/s),fwd(),wheel_tick()\n";
		sprintf(output_msg, "%d,%11.4f,%3d,%10.4f,%3d,%16I64d\n", pak_o1.gps_week, (double)pak_o1.gps_millisecs / 1000.0, pak_o1.mode,
			pak_o1.speed, pak_o1.fwd, pak_o1.wheel_tick);
		FILE* f_o1 = get_file("o1.csv", title);
		if (f_o1) fprintf(f_o1, output_msg);
		//kml
		FILE* f_odo = get_file("odo.txt", title);
		if (f_odo) fprintf(f_odo, output_msg);
		//process
		FILE* f_process = get_file("process");
		if (f_process) fprintf(f_process, "$GPODO,%s", output_msg);
	}

	void RTK350LA_decoder::output_rr() {
		uint8_t rover_len = pak_rr.len;
		FILE*f_rr = get_file("rover_rtcm.bin");
		if (f_rr) fwrite(pak_rr.data, 1, rover_len, f_rr);
	}

    void RTK350LA_decoder::output_pvt() {
		std::string name_csv = "pvt.csv";        
		std::string title = 
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",numberOfSVs(),time(s),position_x(mm),position_y(mm),position_z(mm),velocity_east(mm/s),velocity_north(mm/s),velocity_down(mm/s),position_x_std,position_y_std,position_z_std,velocity_east_std,velocity_north_std,velocity_down_std\n";
		sprintf(output_msg, "%d,%11.4f,%3d,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f\r\n", pak_pvt.week, (double)pak_pvt.timeOfWeek / 1000.0, pak_pvt.nspp_use, pak_pvt.pvt_time, 
        pak_pvt.pvt_pos[0], pak_pvt.pvt_pos[1], pak_pvt.pvt_pos[2], pak_pvt.pvt_pos[3], pak_pvt.pvt_pos[4], pak_pvt.pvt_pos[5],
        pak_pvt.pvt_std[0], pak_pvt.pvt_std[1], pak_pvt.pvt_std[2], pak_pvt.pvt_std[3], pak_pvt.pvt_std[4], pak_pvt.pvt_std[5]);
		FILE* fp_csv = get_file(name_csv, title);
        if(fp_csv) fprintf(fp_csv, output_msg);

		FILE* f_pvt = get_file("pvt.txt", title);
		if (f_pvt) fprintf(f_pvt, output_msg);
		//process
		FILE* f_process = get_file("process");
		if (f_process) fprintf(f_process, "$GPPVT,%s", output_msg);
    }

    void RTK350LA_decoder::output_bias() {
		std::string name_csv = "bias.csv";        
		std::string title = 
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",gyro_bias_x(),gyro_bias_y(),gyro_bias_z(),acc_bias_x(),acc_bias_y(),acc_bias_z()\n";
		sprintf(output_msg, "%d,%11.4f, %10.7f,%10.7f,%10.7f,%10.7f,%10.7f,%10.7f\r\n", pak_bias.week, (double)pak_bias.timeOfWeek / 1000.0,
        pak_bias.gyro_bias_x, pak_bias.gyro_bias_y, pak_bias.gyro_bias_z, pak_bias.acc_bias_x, pak_bias.acc_bias_y, pak_bias.acc_bias_z
        );
		FILE* fp_csv = get_file(name_csv, title);
        if(fp_csv) fprintf(fp_csv, output_msg);

		FILE* f_bias = get_file("bias.txt", title);
		if (f_bias) fprintf(f_bias, output_msg);
		//process
		// FILE* f_process = get_file("process");
		// if (f_process) fprintf(f_process, "$GPBIAS,%s", output_msg);
    }

    void RTK350LA_decoder::output_sT()
    {
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
			, pak_sT.GPS_Week, (double)(pak_sT.GPS_TimeOfWeek) / 1000,
			pak_sT.year, pak_sT.mouth, pak_sT.day, pak_sT.hour, pak_sT.min, pak_sT.sec,
			pak_sT.status_bit.imu_temp_status, pak_sT.status_bit.imu_acce_status, pak_sT.status_bit.imu_gyro_status,
			pak_sT.status_bit.imu_sensor_status1, pak_sT.status_bit.imu_sensor_status2, pak_sT.status_bit.imu_sensor_status3, pak_sT.status_bit.imu_overall_status,
			pak_sT.status_bit.gnss_data_status, pak_sT.status_bit.gnss_signal_status, pak_sT.status_bit.power, pak_sT.status_bit.MCU_status, pak_sT.status_bit.pps_status,
			pak_sT.status_bit.zupt_det, pak_sT.status_bit.odo_used, pak_sT.status_bit.odo_recv,
			pak_sT.status_bit.imu_s1_state, pak_sT.status_bit.imu_s2_state, pak_sT.status_bit.imu_s3_state,
			pak_sT.status_bit.time_valid, pak_sT.status_bit.antenna_sensing, pak_sT.status_bit.gnss_chipset,
			pak_sT.status_bit.post,
			pak_sT.imu_temperature, pak_sT.mcu_temperature);
		if (f_sT) fprintf(f_sT, output_msg);
    }


    void RTK350LA_decoder::output_iS()
    {
		std::string name_csv = "iS.csv";
		std::string title =
			"GPS_Week(),GPS_TimeOfWeek(s)"
			",latitude_std(m),longitude_std(m),height_std(m)"
			",north_velocity_std(m/s),north_velocity_std(m/s),north_velocity_std(m/s),lat_velocity_std(m/s),long_velocity_std(m/s)"
			",roll_std(deg),pitch_std(deg),heading_std(deg)\n";            
		FILE* fp_csv = get_file(name_csv, title);
		sprintf(output_msg,
			"%d,%11.4f"
			",%8.3f,%8.3f,%8.3f"
			",%8.3f,%8.3f,%8.3f"
			",%8.3f,%8.3f"
			",%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n",
			pak_iS.week, (double)pak_iS.timeOfWeek / 1000.0,
			(float)pak_iS.latitude_std / 1000, (float)pak_iS.longitude_std / 1000, (float)pak_iS.height_std / 1000, 
            (float)pak_iS.north_velocity_std / 1000, (float)pak_iS.east_velocity_std / 1000, (float)pak_iS.up_velocity_std / 1000, 
            (float)pak_iS.lat_velocity_std / 1000, (float)pak_iS.long_velocity_std / 1000,
			(float)pak_iS.roll_std / 1000, (float)pak_iS.pitch_std / 1000, (float)pak_iS.heading_std / 1000);
		if (fp_csv) fprintf(fp_csv, output_msg);
    }

	void RTK350LA_decoder::parse_packet_payload()
	{
		uint8_t* payload = raw.buff + 3;
		if (all_type_file_output.find(raw.packet_type) == all_type_file_output.end()) return;
		if (all_type_file_output[raw.packet_type] == 0) return;
		switch (raw.packet_type) {
		case em_S2:
		{
			size_t packet_size = sizeof(out_S2_struct);
			if (raw.length == packet_size) {
				memcpy(&pak_S2, payload, sizeof(out_S2_struct));
				if (!m_isOutputFile) break;
				output_S2();
#ifdef OUTPUT_INNER_FILE
				if (is_pruned)break;
#endif
			}

		}break;
        case em_SI:
		{
			size_t packet_size = sizeof(out_SI_struct);
			if (raw.length == packet_size) {
				memcpy(&pak_SI, payload, sizeof(out_SI_struct));
				if (!m_isOutputFile) break;
				output_SI();
			}

		}break;

		case em_gN:
		{
            if(raw.length == sizeof(output_gN_t)){
				memcpy(&pak_gN, payload, raw.length);
				output_gN();
			}

		}break;
		case em_iN:
		{
			if (raw.length == sizeof(out_iN_struct)) {
				memcpy(&pak_iN, payload, sizeof(out_iN_struct));
				if (!m_isOutputFile) break;
				output_iN();
			}
		}break;
		case em_o1:
		{
			if (raw.length == sizeof(output_o1_t)) {
				memcpy(&pak_o1, payload, sizeof(output_o1_t));
				if (!m_isOutputFile) break;
				output_o1();
			}
		}break;
		case em_pv:
		{
			if (raw.length == sizeof(ucb_pvt_t)) {
				memcpy(&pak_pvt, payload, sizeof(ucb_pvt_t));
				if (!m_isOutputFile) break;
				output_pvt();
			}
		}break;
		case em_rR:
		{
			// if (raw.length == sizeof(ucb_rr_t) - 1) 
            {
                pak_rr.len = raw.length;
				memcpy(&pak_rr.data, payload, raw.length);
				if (!m_isOutputFile) break;
				output_rr();
			}
		}break;
		case em_iB:
		{
			if (raw.length == sizeof(ucb_bias_t)) {
				memcpy(&pak_bias, payload, sizeof(ucb_bias_t));
				if (!m_isOutputFile) break;
				output_bias();
			}
		}break;
        case em_sT:
		{
			if (raw.length == sizeof(output_sT_t)) {
				memcpy(&pak_sT, payload, sizeof(output_sT_t));
				if (!m_isOutputFile) break;
				output_sT();
			}
		}break;
        case em_iS:
		{
			if (raw.length == sizeof(out_iS_struct)) {
				memcpy(&pak_iS, payload, sizeof(out_iS_struct));
				if (!m_isOutputFile) break;
				output_iS();
			}
		}break;
		default:
			break;
		}
	}

	int RTK350LA_decoder::parse_nmea(uint8_t data) {
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
				if (m_isOutputFile && !is_pruned) {
					FILE* f_nmea = get_file("nmea.txt");
					fprintf(f_nmea, (char*)raw.nmea);
				}
				return 2;
			}
		}
		return 0;
	}

	int RTK350LA_decoder::input_raw(uint8_t data)
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
				if (raw.flag == 0 && raw.header[2] == USER_PREAMB) {
					memmove(raw.header, raw.header + 1, 3);
					raw.header_len = 3;
				}
				else {
					raw.header_len = 0;
				}
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

	void RTK350LA_decoder::finish()
	{
		if (!m_isOutputFile) return;
		FILE* f_log = get_file(".log");
		for (std::map<uint16_t, int>::iterator it = all_type_pack_num.begin(); it != all_type_pack_num.end(); it++) {
			fprintf(f_log, "pack_type = 0x%04x, pack_num = %d\n", (uint16_t)it->first, (int)it->second);
		}
		fprintf(f_log, "all_pack_num = %d\ncrc_right_num = %d\ncrc_error_num = %d\n", pack_num, crc_right_num, crc_error_num);
		if (!is_pruned) {
			Kml_Generator::Instance()->open_files(base_file_name);
			Kml_Generator::Instance()->write_files();
			Kml_Generator::Instance()->close_files();
		}
		close_all_files();
	}

	int RTK350LA_decoder::get_current_type()
	{
		return raw.packet_type;
	}
}
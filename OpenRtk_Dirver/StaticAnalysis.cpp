#include "StaticAnalysis.h"
#include "rtcm.h"

StaticAnalysis::StaticAnalysis(QObject *parent)
	: QObject(parent)
{
	imu_1hz_file = NULL;
	imu_g_1hz_file = NULL;
	append_num = 0;
	m_start_line = 0;
	cep_level_thres = 99.7;
	hor_dist_cep_thres = 0.2;
	ver_dist_cep_thres = 0.2;
	hor_vel_cep_thres = 0.2;
	ver_vel_cep_thres = 0.2;
	memset(&m_imu_total, 0, sizeof(m_imu_total));
}

StaticAnalysis::~StaticAnalysis()
{
}

void StaticAnalysis::init()
{
	append_num = 0;
	m_gnss_sol_list.clear();
	memset(&m_imu_total, 0, sizeof(m_imu_total));
	m_raw_imu_list.clear();
}

void StaticAnalysis::set_out_base_name(QString basename)
{
	m_OutBaseName = basename;
}

char * StaticAnalysis::week_2_time_str(int week, uint32_t millisecs)
{
	gtime_t gpstime = gpst2time(week, (double)millisecs / 1000.0);
	gtime_t utctime = gpst2utc(gpstime);
	return time_str(utctime, 2);
}

void StaticAnalysis::append_gnss_sol(static_gnss_t* gnss)
{
	if (append_num >= m_start_line) {
		m_gnss_sol_list.append(*gnss);
	}
	append_num++;
}

void StaticAnalysis::append_gnss_sol_ins401(Ins401_Tool::gnss_sol_t * gnss)
{
	static_gnss_t static_gnss = { 0 };
	static_gnss.gps_week = gnss->gps_week;
	static_gnss.gps_millisecs = gnss->gps_millisecs;
	static_gnss.position_type = gnss->position_type;
	static_gnss.latitude = gnss->latitude;
	static_gnss.longitude = gnss->longitude;
	static_gnss.height = gnss->height;
	static_gnss.north_vel = gnss->north_vel;
	static_gnss.east_vel = gnss->east_vel;
	static_gnss.up_vel = gnss->up_vel;
	append_gnss_sol(&static_gnss);
}

void StaticAnalysis::append_gnss_sol_rtk330la(RTK330LA_Tool::inceptio_gN_t * gnss)
{
	static_gnss_t static_gnss = { 0 };
	static_gnss.gps_week = gnss->GPS_Week;
	static_gnss.gps_millisecs = uint32_t(gnss->GPS_TimeOfWeek*1000);
	static_gnss.position_type = gnss->positionMode;
	static_gnss.latitude = (double)gnss->latitude*180.0 / MAX_INT;
	static_gnss.longitude = (double)gnss->longitude*180.0 / MAX_INT;
	static_gnss.height = gnss->height;
	static_gnss.north_vel = gnss->velocityNorth / 100.0f;
	static_gnss.east_vel = gnss->velocityEast / 100.0f;
	static_gnss.up_vel = gnss->velocityUp / 100.0f;
	append_gnss_sol(&static_gnss);
}

void StaticAnalysis::append_imu_raw(imu_t * imu)
{
	if (imu_1hz_file == NULL) {
		create_file(imu_1hz_file, "imu_1hz.csv",
			"GPS_Week(),GPS_TimeOfWeek(s),x_accel(m/s^2),y_accel(m/s^2),z_accel(m/s^2),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");
	}
	if (imu_g_1hz_file == NULL) {
		create_file(imu_g_1hz_file, "imu_g_1hz.csv",
			"GPS_Week(),GPS_TimeOfWeek(s),x_accel(g),y_accel(g),z_accelg),x_gyro(deg/s),y_gyro(deg/s),z_gyro(deg/s)\n");
	}
	bool is_append = false;
	if (m_imu_total.last_time == 0) {
		m_imu_total.last_time = imu->gps_millisecs;
		m_imu_total.accel[0].max = imu->x_accel;
		m_imu_total.accel[0].min = imu->x_accel;
		m_imu_total.accel[0].sum = imu->x_accel;
		m_imu_total.accel[1].max = imu->y_accel;
		m_imu_total.accel[1].min = imu->y_accel;
		m_imu_total.accel[1].sum = imu->y_accel;
		m_imu_total.accel[2].max = imu->z_accel;
		m_imu_total.accel[2].min = imu->z_accel;
		m_imu_total.accel[2].sum = imu->z_accel;
		m_imu_total.gyro[0].max = imu->x_gyro;
		m_imu_total.gyro[0].min = imu->x_gyro;
		m_imu_total.gyro[0].sum = imu->x_gyro;
		m_imu_total.gyro[1].max = imu->y_gyro;
		m_imu_total.gyro[1].min = imu->y_gyro;
		m_imu_total.gyro[1].sum = imu->y_gyro;
		m_imu_total.gyro[2].max = imu->z_gyro;
		m_imu_total.gyro[2].min = imu->z_gyro;
		m_imu_total.gyro[2].sum = imu->z_gyro;
		is_append = true;
	}
	else {
		if (uint32_t(m_imu_total.last_time / 1000) != uint32_t(imu->gps_millisecs / 1000)) {
			m_imu_total.last_time = imu->gps_millisecs;
			m_imu_total.accel[0].max = max(imu->x_accel, m_imu_total.accel[0].max);
			m_imu_total.accel[0].min = min(imu->x_accel, m_imu_total.accel[0].min);
			m_imu_total.accel[0].sum += (imu->x_accel);
			m_imu_total.accel[1].max = max(imu->y_accel, m_imu_total.accel[1].max);
			m_imu_total.accel[1].min = min(imu->y_accel, m_imu_total.accel[1].min);
			m_imu_total.accel[1].sum += (imu->y_accel);
			m_imu_total.accel[2].max = max(imu->z_accel, m_imu_total.accel[2].max);
			m_imu_total.accel[2].min = min(imu->z_accel, m_imu_total.accel[2].min);
			m_imu_total.accel[2].sum += (imu->z_accel);
			m_imu_total.gyro[0].max = max(imu->x_gyro, m_imu_total.gyro[0].max);
			m_imu_total.gyro[0].min = min(imu->x_gyro, m_imu_total.gyro[0].min);
			m_imu_total.gyro[0].sum += imu->x_gyro;
			m_imu_total.gyro[1].max = max(imu->y_gyro, m_imu_total.gyro[1].max);
			m_imu_total.gyro[1].min = min(imu->y_gyro, m_imu_total.gyro[1].min);
			m_imu_total.gyro[1].sum += imu->y_gyro;
			m_imu_total.gyro[2].max = max(imu->z_gyro, m_imu_total.gyro[2].max);
			m_imu_total.gyro[2].min = min(imu->z_gyro, m_imu_total.gyro[2].min);
			m_imu_total.gyro[2].sum += imu->z_gyro;
			is_append = true;
		}
	}
	if (is_append) {
		fprintf(imu_1hz_file, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu->GPS_Week, (double)imu->gps_millisecs / 1000.0,
			imu->x_accel, imu->y_accel, imu->z_accel, imu->x_gyro, imu->y_gyro, imu->z_gyro);
		fprintf(imu_g_1hz_file, "%d,%11.4f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", imu->GPS_Week, (double)imu->gps_millisecs / 1000.0,
			imu->x_accel / _G_, imu->y_accel / _G_, imu->z_accel / _G_, imu->x_gyro, imu->y_gyro, imu->z_gyro);
		m_raw_imu_list.append(*imu);
	}
}

void StaticAnalysis::append_imu_ins401(Ins401_Tool::raw_imu_t * imu)
{
	imu_t static_imu = { 0 };
	memcpy(&static_imu, imu,sizeof(imu_t));
	append_imu_raw(&static_imu);
}

void StaticAnalysis::append_imu_rtk330la(RTK330LA_Tool::inceptio_s1_t * imu)
{
	imu_t static_imu = { 0 };
	static_imu.GPS_Week = imu->GPS_Week;
	static_imu.gps_millisecs = uint32_t(imu->GPS_TimeOfWeek*1000);
	static_imu.x_accel = imu->x_accel;
	static_imu.y_accel = imu->y_accel;
	static_imu.z_accel = imu->z_accel;
	static_imu.x_gyro = imu->x_gyro;
	static_imu.y_gyro = imu->y_gyro;
	static_imu.z_gyro = imu->z_gyro;
	append_imu_raw(&static_imu);
}

void StaticAnalysis::gnss_summary()
{
	if (m_gnss_sol_list.size() == 0) {
		return;
	}
	int fixed_num = 0;
	int float_num = 0;
	int spp_num = 0;
	int start_line = 0;
	QList<double> latitude_list;
	QList<double> longitude_list;
	QList<double> height_list;
	for (int i = 0; i < m_gnss_sol_list.size(); i++) {
		if (m_gnss_sol_list[i].position_type == 4) {
			break;
		}
		start_line ++;
	}
	for (int i = start_line; i < m_gnss_sol_list.size(); i++) {
		if (m_gnss_sol_list[i].position_type == 4) {
			latitude_list.append(m_gnss_sol_list[i].latitude);
			longitude_list.append(m_gnss_sol_list[i].longitude);
			height_list.append(m_gnss_sol_list[i].height);
			fixed_num++;
		}
		else if (m_gnss_sol_list[i].position_type == 5) {
			float_num++;
		}
		else if (m_gnss_sol_list[i].position_type == 1) {
			spp_num++;
		}
	}
	if (height_list.size() == 0) {
		return;
	}
	qSort(latitude_list);
	qSort(longitude_list);
	qSort(height_list);

	int median_index = latitude_list.size() / 2;
	double median_pos[3] = { 0 };
	median_pos[0] = latitude_list[median_index];
	median_pos[1] = longitude_list[median_index];
	median_pos[2] = height_list[median_index];
	latitude_list.clear();
	longitude_list.clear();
	height_list.clear();

	median_pos[0] = median_pos[0] * D2R;
	median_pos[1] = median_pos[1] * D2R;
	double median_xyz[3] = { 0 };
	pos2ecef(median_pos, median_xyz);

	//QList<double> dist_list;
	QList<double> hor_dist_list;
	QList<double> ver_dist_list;
	QList<double> hor_vel_list;
	QList<double> ver_vel_list;
	FILE* diff_file = NULL;
	std::string title =
		"DateTime(),GPS_Week(),GPS_TimeOfWeek(s),position_type()"
		",latitude(deg),longitude(deg),height(m)"
		",hor_error(m),ver_error(m)"
		",north_vel(m/s),east_vel(m/s),up_vel(m/s)"
		",hor_vel(m/s),ver_vel(m/s)\n";
	create_file(diff_file, "gnss_diff.csv", title.c_str());
	int hor_dist_pass = 0;
	int ver_dist_pass = 0;
	int hor_vel_pass = 0;
	int ver_vel_pass = 0;	
	for (int i = start_line; i < m_gnss_sol_list.size(); i++) {
		//if (m_gnss_sol_list[i].position_type != 4) continue;
		static_gnss_t gnss = m_gnss_sol_list[i];
		double pos[3] = { 0 };
		pos[0] = gnss.latitude;
		pos[1] = gnss.longitude;
		pos[2] = gnss.height;
		double ver_dist = fabs(pos[2] - median_pos[2]);
		ver_dist_list.append(ver_dist);

		pos[0] = pos[0] * D2R;
		pos[1] = pos[1] * D2R;
		double xyz[3] = { 0 };
		double d_xyz[3] = { 0 };

		pos[2] = median_pos[2];
		double xyz_hor[3] = { 0 };
		pos2ecef(pos, xyz_hor);
		d_xyz[0] = xyz_hor[0] - median_xyz[0];
		d_xyz[1] = xyz_hor[1] - median_xyz[1];
		d_xyz[2] = xyz_hor[2] - median_xyz[2];
		double hor_dist = sqrt(d_xyz[0] * d_xyz[0] + d_xyz[1] * d_xyz[1] + d_xyz[2] * d_xyz[2]);
		hor_dist_list.append(hor_dist);

		double hor_vel = sqrt(gnss.north_vel*gnss.north_vel + gnss.east_vel*gnss.east_vel);
		hor_vel_list.append(hor_vel);
		double ver_vel = fabs(gnss.up_vel);
		ver_vel_list.append(ver_vel);

		if (diff_file)
		{
			fprintf(diff_file, "%s,", week_2_time_str(gnss.gps_week, gnss.gps_millisecs));
			fprintf(diff_file, "%d,%11.4f,%3d", gnss.gps_week, (double)gnss.gps_millisecs / 1000.0, gnss.position_type);
			fprintf(diff_file, ",%14.9f,%14.9f,%10.4f", gnss.latitude, gnss.longitude, gnss.height);
			fprintf(diff_file, ",%9.4f,%9.4f", hor_dist, ver_dist);
			fprintf(diff_file, ",%10.4f,%10.4f,%10.4f", gnss.north_vel, gnss.east_vel, gnss.up_vel);
			fprintf(diff_file, ",%10.4f,%10.4f\n", hor_vel, ver_vel);
		}


		if (hor_dist < hor_dist_cep_thres) {
			hor_dist_pass++;
		}
		if (ver_dist < ver_dist_cep_thres) {
			ver_dist_pass++;
		}
		if (hor_vel < hor_vel_cep_thres) {
			hor_vel_pass++;
		}
		if (ver_vel < ver_vel_cep_thres) {
			ver_vel_pass++;
		}
	}
	if(diff_file) fclose(diff_file);
	//qSort(dist_list);
	qSort(hor_dist_list);
	qSort(ver_dist_list);
	qSort(hor_vel_list);
	qSort(ver_vel_list);
	double cep_level = cep_level_thres / 100;
	int cep_index = hor_dist_list.size() * cep_level - 1;
	double hor_dist_cep = hor_dist_list[cep_index];
	double ver_dist_cep = ver_dist_list[cep_index];
	double hor_vel_cep = hor_vel_list[cep_index];
	double ver_vel_cep = ver_vel_list[cep_index];
	double hor_dist_pass_percent = (double)hor_dist_pass / (double)hor_dist_list.size() * 100.0;
	double ver_dist_pass_percent = (double)ver_dist_pass / (double)ver_dist_list.size() * 100.0;
	double hor_vel_pass_percent = (double)hor_vel_pass / (double)hor_vel_list.size() * 100.0;
	double ver_vel_pass_percent = (double)ver_vel_pass / (double)ver_vel_list.size() * 100.0;
	FILE* total_file = NULL;
	create_file(total_file, "gnss_total.csv", NULL);
	if (total_file) {
		fprintf(total_file, "cep_thres(%%): %4.1f\n", cep_level_thres);
		fprintf(total_file, "hor_pos_err_thres(m): %4.1f\n", hor_dist_cep_thres);
		fprintf(total_file, "ver_pos_err_thres(m): %4.1f\n", ver_dist_cep_thres);
		fprintf(total_file, "hor_vel_err_thres(m/s): %4.1f\n", hor_vel_cep_thres);
		fprintf(total_file, "ver_vel_err_thres(m/s): %4.1f\n", ver_vel_cep_thres);
		fprintf(total_file, "start line: %d\n", start_line);
		fprintf(total_file, "median pos(m): %14.10f,%14.10f,%14.10f\n", median_pos[0] * R2D, median_pos[1] * R2D, median_pos[2]);
		fprintf(total_file, "\n");
		fprintf(total_file, "%16s,%16s,%16s,%16s,%16s,%16s,%16s\n", "hor_pos_err(m)", "ver_pos_err(m)", "hor_vel_err(m/s)", "ver_vel_err(m/s)", "fixed num", "float num", "spp num");
		fprintf(total_file, "%16.3f,%16.3f,%16.3f,%16.3f,%16d,%16d,%16d\n", hor_dist_cep, ver_dist_cep, hor_vel_cep, ver_vel_cep, fixed_num, float_num, spp_num);
		fprintf(total_file, "%15.2f%%,%15.2f%%,%15.2f%%,%15.2f%%\n", hor_dist_pass_percent, ver_dist_pass_percent, hor_vel_pass_percent, ver_vel_pass_percent);
		fprintf(total_file, "%16d,%16d,%16d,%16d\n", hor_dist_pass, ver_dist_pass, hor_vel_pass, ver_vel_pass);
		fprintf(total_file, "\n");
		if (hor_dist_cep <= hor_dist_cep_thres && ver_dist_cep <= ver_dist_cep_thres &&
			hor_vel_cep <= hor_vel_cep_thres && ver_vel_cep <= ver_vel_cep_thres) {
			fprintf(total_file, "pass\n");
		}
		else {
			fprintf(total_file, "failed\n");
		}
		fclose(total_file);
	}
}

void StaticAnalysis::set_thres(double cep_level, double hor_dist_cep, double ver_dist_cep, double hor_vel_cep, double ver_vel_cep, int32_t start_line)
{
	cep_level_thres = cep_level;
	hor_dist_cep_thres = hor_dist_cep;
	ver_dist_cep_thres = ver_dist_cep;
	hor_vel_cep_thres = hor_vel_cep;
	ver_vel_cep_thres = ver_vel_cep;
	m_start_line = start_line;
}

void StaticAnalysis::create_file(FILE *& file, const char * suffix, const char * title)
{
	if (file == NULL) {
		char file_name[256] = { 0 };
		sprintf(file_name, "%s_%s", m_OutBaseName.toLocal8Bit().data(), suffix);
		file = fopen(file_name, "wb");
		if (file && title) fprintf(file, title);
	}
}

void StaticAnalysis::summary()
{
	gnss_summary();
	imu_summary();
}

void StaticAnalysis::imu_summary()
{
	if (imu_1hz_file) fclose(imu_1hz_file); imu_1hz_file = NULL;
	if (imu_g_1hz_file) fclose(imu_g_1hz_file); imu_g_1hz_file = NULL;

	for (int i = 0; i < 3; i++) {
		m_imu_total.accel[i].avg = m_imu_total.accel[i].sum / m_raw_imu_list.size();
	}
	for (int i = 0; i < 3; i++) {
		m_imu_total.gyro[i].avg = m_imu_total.gyro[i].sum / m_raw_imu_list.size();
	}
	double diff = 0;
	for (int i = 0; i < m_raw_imu_list.size(); i++) {
		diff = m_raw_imu_list[i].x_accel - m_imu_total.accel[0].avg;
		m_imu_total.accel[0].square_diff_sum += (diff*diff);
		diff = m_raw_imu_list[i].y_accel - m_imu_total.accel[1].avg;
		m_imu_total.accel[1].square_diff_sum += (diff*diff);
		diff = m_raw_imu_list[i].z_accel - m_imu_total.accel[2].avg;
		m_imu_total.accel[2].square_diff_sum += (diff*diff);

		diff = m_raw_imu_list[i].x_gyro - m_imu_total.gyro[0].avg;
		m_imu_total.gyro[0].square_diff_sum += (diff*diff);
		diff = m_raw_imu_list[i].y_gyro - m_imu_total.gyro[1].avg;
		m_imu_total.gyro[1].square_diff_sum += (diff*diff);
		diff = m_raw_imu_list[i].z_gyro - m_imu_total.gyro[2].avg;
		m_imu_total.gyro[2].square_diff_sum += (diff*diff);
	}
	for (int i = 0; i < 3; i++) {
		m_imu_total.accel[i].std = sqrt(m_imu_total.accel[i].square_diff_sum / m_raw_imu_list.size());
	}
	for (int i = 0; i < 3; i++) {
		m_imu_total.gyro[i].std = sqrt(m_imu_total.gyro[i].square_diff_sum / m_raw_imu_list.size());
	}

	FILE* imu_total_file = NULL;
	create_file(imu_total_file, "imu_total.csv", NULL);
	fprintf(imu_total_file, " ,MAX,MIN,AVG,STD\n");
	fprintf(imu_total_file, "x_accel(g),%10.5f,%10.5f,%10.5f,%10.5f\n", m_imu_total.accel[0].max / _G_, m_imu_total.accel[0].min / _G_, m_imu_total.accel[0].avg / _G_, m_imu_total.accel[0].std / _G_);
	fprintf(imu_total_file, "y_accel(g),%10.5f,%10.5f,%10.5f,%10.5f\n", m_imu_total.accel[1].max / _G_, m_imu_total.accel[1].min / _G_, m_imu_total.accel[1].avg / _G_, m_imu_total.accel[1].std / _G_);
	fprintf(imu_total_file, "z_accel(g),%10.5f,%10.5f,%10.5f,%10.5f\n", m_imu_total.accel[2].max / _G_, m_imu_total.accel[2].min / _G_, m_imu_total.accel[2].avg / _G_, m_imu_total.accel[2].std / _G_);
	fprintf(imu_total_file, "x_gyro,%10.5f,%10.5f,%10.5f,%10.5f\n", m_imu_total.gyro[0].max, m_imu_total.gyro[0].min, m_imu_total.gyro[0].avg, m_imu_total.gyro[0].std);
	fprintf(imu_total_file, "y_gyro,%10.5f,%10.5f,%10.5f,%10.5f\n", m_imu_total.gyro[1].max, m_imu_total.gyro[1].min, m_imu_total.gyro[1].avg, m_imu_total.gyro[1].std);
	fprintf(imu_total_file, "z_gyro,%10.5f,%10.5f,%10.5f,%10.5f\n", m_imu_total.gyro[2].max, m_imu_total.gyro[2].min, m_imu_total.gyro[2].avg, m_imu_total.gyro[2].std);
	fclose(imu_total_file);
}
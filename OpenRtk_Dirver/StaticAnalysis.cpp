#include "StaticAnalysis.h"
#include "rtcm.h"

StaticAnalysis::StaticAnalysis(QObject *parent)
	: QObject(parent)
{
	imu_1hz_file = NULL;
	imu_g_1hz_file = NULL;
	append_num = 0;
	m_start_line = 30;
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
	m_gnss_sol_list.clear();
	memset(&m_imu_total, 0, sizeof(m_imu_total));
	m_raw_imu_list.clear();
}

void StaticAnalysis::set_out_base_name(QString basename)
{
	m_OutBaseName = basename;
}

void StaticAnalysis::append_gnss_sol(static_gnss_t* gnss)
{
	if (append_num >= m_start_line) {
		m_gnss_sol_list.append(*gnss);
	}
	append_num++;
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

void StaticAnalysis::static_point_cep()
{
	if (m_gnss_sol_list.size() == 0) {
		return;
	}
	int fixed_num = 0;
	int float_num = 0;
	int spp_num = 0;
	QList<double> latitude_list;
	QList<double> longitude_list;
	QList<double> height_list;
	for (int i = 0; i < m_gnss_sol_list.size(); i++) {
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
	for (int i = 0; i < m_gnss_sol_list.size(); i++) {
		double pos[3] = { 0 };
		pos[0] = m_gnss_sol_list[i].latitude;
		pos[1] = m_gnss_sol_list[i].longitude;
		pos[2] = m_gnss_sol_list[i].height;
		ver_dist_list.append(fabs(pos[2] - median_pos[2]));

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
		double distance = sqrt(d_xyz[0] * d_xyz[0] + d_xyz[1] * d_xyz[1] + d_xyz[2] * d_xyz[2]);
		hor_dist_list.append(distance);

		double hor_vel = sqrt(m_gnss_sol_list[i].north_vel*m_gnss_sol_list[i].north_vel + m_gnss_sol_list[i].east_vel*m_gnss_sol_list[i].east_vel);
		hor_vel_list.append(hor_vel);
		ver_vel_list.append(fabs(m_gnss_sol_list[i].up_vel));
	}
	//qSort(dist_list);
	qSort(hor_dist_list);
	qSort(ver_dist_list);
	qSort(hor_vel_list);
	qSort(ver_vel_list);
	double cep_level = cep_level_thres / 100;
	int cep_index = m_gnss_sol_list.size()*cep_level - 1;
	double hor_dist_cep = hor_dist_list[cep_index];
	double ver_dist_cep = ver_dist_list[cep_index];
	double hor_vel_cep = hor_vel_list[cep_index];
	double ver_vel_cep = ver_vel_list[cep_index];
	FILE* total_file = NULL;
	create_file(total_file, "gnss_total.csv", NULL);
	if (total_file) {
		fprintf(total_file, "cep_thres(%%): %4.1f\n", cep_level_thres);
		fprintf(total_file, "hor_pos_err_thres(m): %4.1f\n", hor_dist_cep_thres);
		fprintf(total_file, "ver_pos_err_thres(m): %4.1f\n", ver_dist_cep_thres);
		fprintf(total_file, "hor_vel_err_thres(m/s): %4.1f\n", hor_vel_cep_thres);
		fprintf(total_file, "ver_vel_err_thres(m/s): %4.1f\n", ver_vel_cep_thres);
		fprintf(total_file, "start line: %d\n", m_start_line);
		fprintf(total_file, "median pos(m): %14.10f,%14.10f,%14.10f\n", median_pos[0] * R2D, median_pos[1] * R2D, median_pos[2]);
		fprintf(total_file, "\n");
		fprintf(total_file, "%16s,%16s,%16s,%16s,%16s,%16s,%16s\n", "hor_pos_err(m)", "ver_pos_err(m)", "hor_vel_err(m/s)", "ver_vel_err(m/s)", "fixed num", "float num", "spp num");
		fprintf(total_file, "%16.3f,%16.3f,%16.3f,%16.3f,%16d,%16d,%16d\n", hor_dist_cep, ver_dist_cep, hor_vel_cep, ver_vel_cep, fixed_num, float_num, spp_num);
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
	append_num = 0;
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
	static_point_cep();
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
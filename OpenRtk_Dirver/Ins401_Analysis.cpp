#include "Ins401_Analysis.h"
#include "rtcm.h"

Ins401_Analysis::Ins401_Analysis(QObject *parent)
	: QObject(parent)
{
	m_static_point_ecp = true;
	cep_level_thres = 99.7;
	hor_dist_cep_thres = 0.1;
	ver_dist_cep_thres = 0.1;
	hor_vel_cep_thres = 0.1;
	ver_vel_cep_thres = 0.1;
}

Ins401_Analysis::~Ins401_Analysis()
{
}

void Ins401_Analysis::init() {
	m_gnss_sol_list.clear();
}

void Ins401_Analysis::set_out_base_name(QString basename)
{
	m_OutBaseName = basename;
}

void Ins401_Analysis::append_gnss_sol(Ins401_Tool::gnss_sol_t* gnss) {
	m_gnss_sol_list.append(*gnss);
}

void Ins401_Analysis::static_point_cep()
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

	int median_index = latitude_list.size() / 2 - 1;
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
		//pos2ecef(pos, xyz);		
		//d_xyz[0] = xyz[0] - median_xyz[0];
		//d_xyz[1] = xyz[1] - median_xyz[1];
		//d_xyz[2] = xyz[2] - median_xyz[2];
		//double distance = sqrt(d_xyz[0] * d_xyz[0] + d_xyz[1] * d_xyz[1] + d_xyz[2] * d_xyz[2]);
		//dist_list.append(distance);

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
	double cep_level = cep_level_thres /100;
	int cep_index = m_gnss_sol_list.size()*cep_level - 1;
	//double dist_cep997 = dist_list[cep997_index];
	double hor_dist_cep = hor_dist_list[cep_index];
	double ver_dist_cep = ver_dist_list[cep_index];
	double hor_vel_cep = hor_vel_list[cep_index];
	double ver_vel_cep = ver_vel_list[cep_index];
	char file_name[256] = { 0 };
	sprintf(file_name, "%s_total.csv", m_OutBaseName.toLocal8Bit().data());
	FILE* total_file = fopen(file_name, "w");
	if (total_file) {
		fprintf(total_file, "cep_thres(%): %4.1f\n", cep_level_thres);
		fprintf(total_file, "hor_pos_err_thres(m): %4.1f\n", hor_dist_cep_thres);
		fprintf(total_file, "ver_pos_err_thres(m): %4.1f\n", ver_dist_cep_thres);
		fprintf(total_file, "hor_vel_err_thres(m/s): %4.1f\n", hor_vel_cep_thres);
		fprintf(total_file, "ver_vel_err_thres(m/s): %4.1f\n", ver_vel_cep_thres);
		fprintf(total_file, "\n");
		fprintf(total_file, "%16s,%16s,%16s,%16s,%16s,%16s,%16s\n", "hor_pos_err(m)", "ver_pos_err(m)", "hor_vel_err(m/s)", "ver_vel_err(m/s)","fixed num", "float num", "spp num");
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

void Ins401_Analysis::set_thres(double cep_level, double hor_dist_cep, double ver_dist_cep, double hor_vel_cep, double ver_vel_cep)
{
	cep_level_thres = cep_level;
	hor_dist_cep_thres = hor_dist_cep;
	ver_dist_cep_thres = ver_dist_cep;
	hor_vel_cep_thres = hor_vel_cep;
	ver_vel_cep_thres = ver_vel_cep;
}

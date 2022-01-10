#pragma once

#include <QObject>
#include "ins401.h"
#include "Analysis_Define.h"

namespace Ins401_Tool {

class Ins401_Analysis : public QObject
{
	Q_OBJECT

public:
	Ins401_Analysis(QObject *parent);
	~Ins401_Analysis();
	void init();
	void set_out_base_name(QString basename);
	void append_gnss_sol(gnss_sol_t * gnss);
	void append_imu_raw(raw_imu_t * imu);
	void static_point_cep();
	void set_thres(double cep_level, double hor_dist_cep, double ver_dist_cep, double hor_vel_cep, double ver_vel_cep, int32_t start_line);
	void create_file(FILE * &file, const char * suffix, const char * title);
	void summary();
	void imu_summary();
private:
	QList<gnss_sol_t> m_gnss_sol_list;
	QString m_OutBaseName;
	double cep_level_thres;
	double hor_dist_cep_thres;
	double ver_dist_cep_thres;
	double hor_vel_cep_thres;
	double ver_vel_cep_thres;
	int32_t append_num;
	int32_t m_start_line;
	QList<raw_imu_t> m_raw_imu_list;
	imu_total_t m_imu_total;
	FILE* imu_1hz_file;
	FILE* imu_g_1hz_file;
};

}
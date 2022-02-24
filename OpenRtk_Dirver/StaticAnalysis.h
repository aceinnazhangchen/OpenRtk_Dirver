#pragma once

#include <QObject>
#include "common.h"
#include "Analysis_Define.h"
#include "ins401.h"
#include "openrtk_inceptio.h"

class StaticAnalysis : public QObject
{
	Q_OBJECT

public:
	StaticAnalysis(QObject *parent);
	~StaticAnalysis();
	void init();
	void set_out_base_name(QString basename);
	char * week_2_time_str(int week, uint32_t millisecs);
	void append_gnss_sol(static_gnss_t * gnss);
	void append_gnss_sol_ins401(Ins401_Tool::gnss_sol_t * gnss);
	void append_gnss_sol_rtk330la(RTK330LA_Tool::inceptio_gN_t * gnss);
	void append_imu_raw(imu_t * imu);
	void append_imu_ins401(Ins401_Tool::raw_imu_t * imu);
	void append_imu_rtk330la(RTK330LA_Tool::inceptio_s1_t * imu);
	void set_thres(double cep_level, double hor_dist_cep, double ver_dist_cep, double hor_vel_cep, double ver_vel_cep, int32_t start_line);
	void summary();
	void gnss_summary();
	void imu_summary();
protected:
	void create_file(FILE * &file, const char * suffix, const char * title);
private:
	QList<static_gnss_t> m_gnss_sol_list;
	QString m_OutBaseName;
	double cep_level_thres;
	double hor_dist_cep_thres;
	double ver_dist_cep_thres;
	double hor_vel_cep_thres;
	double ver_vel_cep_thres;
	int32_t append_num;
	int32_t m_start_line;
	QList<imu_t> m_raw_imu_list;
	imu_total_t m_imu_total;
	FILE* imu_1hz_file;
	FILE* imu_g_1hz_file;
};

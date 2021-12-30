#pragma once

#include <QObject>
#include "openrtk_inceptio.h"
#include "Analysis_Define.h"

namespace RTK330LA_Tool {

	class RTK330LA_Analysis : public QObject
	{
		Q_OBJECT

	public:
		RTK330LA_Analysis(QObject *parent);
		~RTK330LA_Analysis();
		void init();
		void set_out_base_name(QString basename);
		void append_gnss_sol(inceptio_gN_t * gnss);
		void append_imu_raw(inceptio_s1_t * imu);
		void static_point_cep();
		void set_thres(double cep_level, double hor_dist_cep, double ver_dist_cep, double hor_vel_cep, double ver_vel_cep, int32_t start_line);
		void create_file(FILE * &file, const char * suffix, const char * title);
		void summary();
		void imu_summary();
	private:
		QList<inceptio_gN_t> m_gnss_sol_list;
		bool m_static_point_ecp;
		QString m_OutBaseName;
		double cep_level_thres;
		double hor_dist_cep_thres;
		double ver_dist_cep_thres;
		double hor_vel_cep_thres;
		double ver_vel_cep_thres;
		int32_t append_num;
		int32_t m_start_line;
		QList<inceptio_s1_t> m_raw_imu_list;
		imu_total_t m_imu_total;
		FILE* imu_1hz_file;
		FILE* imu_g_1hz_file;
	};

}
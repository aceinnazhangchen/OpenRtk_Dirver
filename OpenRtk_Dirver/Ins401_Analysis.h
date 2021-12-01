#pragma once

#include <QObject>
#include "ins401.h"

class Ins401_Analysis : public QObject
{
	Q_OBJECT

public:
	Ins401_Analysis(QObject *parent);
	~Ins401_Analysis();
	void init();
	void set_out_base_name(QString basename);
	void append_gnss_sol(Ins401_Tool::gnss_sol_t * gnss);
	void static_point_cep();
	void set_thres(double cep_level, double hor_dist_cep, double ver_dist_cep, double hor_vel_cep, double ver_vel_cep);
private:
	QList<Ins401_Tool::gnss_sol_t> m_gnss_sol_list;
	bool m_static_point_ecp;
	QString m_OutBaseName;
	double cep_level_thres;
	double hor_dist_cep_thres;
	double ver_dist_cep_thres;
	double hor_vel_cep_thres;
	double ver_vel_cep_thres;
};

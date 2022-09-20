#include "CalculationThread.h"
#include "CalculationCall.h"
#include <QDir>
#include <QFileInfo>
#include "calc_heading.h"

CalculationThread::CalculationThread(QObject *parent, std::vector<stTimeSlice>& timeSlice)
	: QThread(parent)
	, m_TimeSlice(timeSlice)
	, m_isStop(false)
	, m_run_mode(emRunMode_All)
{
	memset(&m_avg_angle, 0, sizeof(m_avg_angle));
}

CalculationThread::~CalculationThread()
{}

void CalculationThread::run()
{
	m_isStop = false;
	CalculateAll();
	emit sgnFinished();
}

void CalculationThread::stop()
{
	m_isStop = true;
}

void CalculationThread::setRunMode(emRunMode mode)
{
	m_run_mode = mode;
}

void CalculationThread::setPostInsFilePath(QString file_path)
{
	m_PostInsFilePath = file_path;
}

void CalculationThread::setGnssposvelFilePath(QString file_path)
{
	m_GnssposvelFilePath = file_path;
}

void CalculationThread::setPostMovbsFilePath(QString file_path)
{
	m_PostMovbsFilePath = file_path;
}

void CalculationThread::setTimeRange(QString week, QString starttime, QString endtime)
{
	m_week_str = week;
	m_starttime_str = starttime;
	m_endtime_str = endtime;
}

stAngle& CalculationThread::getAvgAngle()
{
	return m_avg_angle;
}

void CalculationThread::CalculateAll()
{

	if (m_PostInsFilePath.isEmpty()) return;
	QFileInfo post_ins_file(m_PostInsFilePath);
	if (!post_ins_file.isFile()) return;

	if (m_GnssposvelFilePath.isEmpty()) return;
	QFileInfo gnss_pos_vel_file(m_GnssposvelFilePath);
	if (!gnss_pos_vel_file.isFile()) return;

	QString gnss_dir_path = gnss_pos_vel_file.absolutePath();
	QDir out_perfix_dir = gnss_dir_path + QDir::separator() + "solvemisalign";
	if (!out_perfix_dir.exists()) {
		out_perfix_dir.mkpath(out_perfix_dir.absolutePath());
	}
	angle_list.clear();
	memset(&m_avg_angle, 0, sizeof(stAngle));
	if (m_run_mode == emRunMode_All) {
		QStringList angle_out_file_list;
		for (int i = 0; i < m_TimeSlice.size(); i++) {
			if (m_isStop) break;
			m_week_str = QString::number(m_TimeSlice[i].week);
			m_starttime_str = QString::number(m_TimeSlice[i].starttime / 1000);
			m_endtime_str = QString::number(m_TimeSlice[i].endtime / 1000);

			QString out_perfix_basename = "misalign_" + m_starttime_str + "-" + m_endtime_str;
			QString out_perfix_path = out_perfix_dir.absolutePath() + QDir::separator() + out_perfix_basename;

			CalculationCall::call_dr_mountangle_start(m_PostInsFilePath, m_week_str, m_starttime_str, m_endtime_str, out_perfix_path);

			QString angle_out_file_path = out_perfix_path + "_content_misalign.txt";
			angle_out_file_list << angle_out_file_path;
		}
		QThread::sleep(2);
		foreach(auto angle_out_file, angle_out_file_list) {
			readAngleFromOutFile(angle_out_file);
		}
	}
	else if (m_run_mode == emRunMode_One) {
		QString out_perfix_basename = "misalign_" + m_starttime_str + "-" + m_endtime_str;
		QString out_perfix_path = out_perfix_dir.absolutePath() + QDir::separator() + out_perfix_basename;

		CalculationCall::call_dr_mountangle_start(m_PostInsFilePath, m_week_str, m_starttime_str, m_endtime_str, out_perfix_path);

		QString angle_out_file_path = out_perfix_path + "_content_misalign.txt";
		QThread::sleep(1);
		readAngleFromOutFile(angle_out_file_path);
		if (angle_list.size() == 1) {
			m_avg_angle = angle_list[0];
		}
	}
	if (m_run_mode == emRunMode_All || m_run_mode == emRunMode_Avg) {
		readHeadingFromIns(m_PostInsFilePath);
		readGnssPosVel(m_GnssposvelFilePath);
		//readMovbsTxt();
		readPostMovbsTxt(m_PostMovbsFilePath);
		CalculateAverageAngle();
	}
}

void CalculationThread::readAngleFromOutFile(QString file_path)
{
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_out) {
		char line[256] = { 0 };
		stAngle angle = { 0 };
		int index = 0;
		while (fgets(line, 256, f_out) != 0) {
			index++;
			if (index == 2) {
				QString value = QString(line).trimmed();
				value = value.left(value.size() - 1);
				angle.roll = value.toFloat();
			}
			if (index == 3) {
				QString value = QString(line).trimmed();
				value = value.left(value.size() - 1);
				angle.pitch = value.toFloat();
			}
			if (index == 4) {
				QString value = QString(line).trimmed();
				angle.heading = value.toFloat();
			}
		}
		angle_list.push_back(angle);
		fclose(f_out);
	}
}

void CalculationThread::readHeadingFromGnss(QString file_path)
{
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	char line[256] = { 0 };
	if (f_out) {
		while (fgets(line, 256, f_out) != NULL) {
		}
		if (angle_list.size() > 0) {
			angle_list[angle_list.size() - 1].gnss_heading = atof(line);//读取最后一行
		}
		fclose(f_out);
	}
}

void CalculationThread::readHeadingFromIns(QString file_path) {
	QFileInfo out_file(file_path);
	if (!out_file.isFile()) return;
	FILE* f_out = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_out) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double sum_heading = 0.0;
		double sum_roll = 0.0;
		int heading_num = 0;
		int roll_num = 0;
		while (fgets(line, 256, f_out) != NULL) {
			line_num++;
			if (line_num == 1) continue;
			QString q_line(line);
			QStringList items = q_line.split(",");
			if (items.size() < 13) continue;
			uint32_t time = (uint32_t)(items[1].trimmed().toDouble() * 1000);
			if (time < m_TimeSlice[time_index].starttime)
				continue;
			if (time > m_TimeSlice[time_index].endtime) {
				if (time_index < angle_list.size()) {
					if (heading_num > 0) {
						angle_list[time_index].ins_heading = sum_heading / heading_num;
						if (angle_list[time_index].ins_heading < 0) {
							angle_list[time_index].ins_heading += 360;
						}
					}
					if (roll_num > 0) {
						angle_list[time_index].ins_roll = sum_roll / roll_num;
					}
				}
				time_index++;
				sum_heading = 0.0;
				sum_roll = 0.0;
				heading_num = 0;
				roll_num = 0;
				if (time_index >= m_TimeSlice.size())
					break;
			}
			if (time % 1000 != 0)
				continue;
			if (time >= m_TimeSlice[time_index].starttime && time < m_TimeSlice[time_index].endtime) {
				double heading = items[10].trimmed().toDouble();
				double roll = items[8].trimmed().toDouble();
				if (heading < 0) {
					heading += 360;
				}
				sum_heading += heading;
				heading_num++;
				sum_roll += roll;
				roll_num++;
			}
		}
		fclose(f_out);
	}
}

void CalculationThread::readGnssPosVel(QString file_path) {
	QFileInfo file_info(file_path);
	if (!file_info.isFile()) return;
	FILE* f_gnssposvel = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_gnssposvel) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double cur_pos[3] = { 0 };
		double last_pos[3] = { 0 };
		double sum_heading = 0.0;
		double sum_heading_calc = 0.0;
		int heading_num = 0;
		double last_time = 0.0;
		while (fgets(line, 256, f_gnssposvel) != NULL) {
			line_num++;
			QString q_line(line);
			QStringList items = q_line.split(",");
			if (items.size() < 13)
				continue;
			double cur_time = items[1].trimmed().toDouble();
			uint32_t time = (uint32_t)(cur_time * 1000);
			if (time < m_TimeSlice[time_index].starttime)
				continue;
			if (time > m_TimeSlice[time_index].endtime) {
				if (time_index < angle_list.size()) {
					if (heading_num > 0) {
						//统计平均heading
						angle_list[time_index].gnssposvel_heading = sum_heading / heading_num;
						if (angle_list[time_index].gnssposvel_heading < 0) {
							angle_list[time_index].gnssposvel_heading += 360;
						}
						angle_list[time_index].gnss_heading = sum_heading_calc / heading_num;
						if (angle_list[time_index].gnss_heading < 0) {
							angle_list[time_index].gnss_heading += 360;
						}
					}
				}
				time_index++;//递增区间id
				sum_heading = 0.0;
				sum_heading_calc = 0.0;
				heading_num = 0;
				if (time_index >= m_TimeSlice.size())//最后一个区间计算完成
					break;
			}
			if (time % 1000 != 0)
				continue;
			if (time >= m_TimeSlice[time_index].starttime && time < m_TimeSlice[time_index].endtime) {//累计区间内的heading
				int type = items[8].trimmed().toInt();
				if (type == 4) {
					double heading = items[12].trimmed().toDouble();
					if (heading < 0) {
						heading += 360;
					}
					cur_pos[0] = items[3].trimmed().toDouble();
					cur_pos[1] = items[4].trimmed().toDouble();
					cur_pos[2] = items[5].trimmed().toDouble();
					double heading_calc = get_heading(last_pos, cur_pos, cur_time - last_time);	
					heading_calc = heading_calc * 180 / PI;
					if (heading_calc < 0) {
						heading_calc += 360;
					}
					sum_heading += heading;
					sum_heading_calc += heading;
					heading_num++;
					last_time = cur_time;
					memcpy(last_pos, cur_pos, 3 * sizeof(double));
				}
			}
		}
		fclose(f_gnssposvel);
	}
}

void CalculationThread::readMovbsTxt(QString file_path) {
	QFileInfo file_info(file_path);
	if (!file_info.isFile()) return;
	FILE* f_movbs = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_movbs) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double sum_heading = 0.0;
		int heading_num = 0;
		while (fgets(line, 256, f_movbs) != NULL) {
			line_num++;
			QString q_line(line);
			QStringList items = q_line.split(",");
			if (items.size() < 10)
				continue;
			uint32_t time = (uint32_t)(items[2].trimmed().toDouble() * 1000);
			if (time < m_TimeSlice[time_index].starttime)
				continue;
			if (time > m_TimeSlice[time_index].endtime) {
				if (time_index < angle_list.size()) {
					if (heading_num > 0) {
						//统计平均heading
						angle_list[time_index].movbs_heading = sum_heading / heading_num;
						if (angle_list[time_index].movbs_heading < 0) {
							angle_list[time_index].movbs_heading += 360;
						}
					}
				}
				time_index++;//递增区间id
				sum_heading = 0.0;
				heading_num = 0;
				if (time_index >= m_TimeSlice.size())//最后一个区间计算完成
					break;
			}
			if (time >= m_TimeSlice[time_index].starttime && time < m_TimeSlice[time_index].endtime) {//累计区间内的heading
				int type1 = items[5].trimmed().toInt();
				int type2 = items[6].trimmed().toInt();
				if (type1 == 4 && type2 == 4) {
					double heading = items[9].trimmed().toDouble();
					if (heading < 0) {
						heading += 360;
					}
					sum_heading += heading;
					heading_num++;
				}
			}
		}
		fclose(f_movbs);
	}
}

void CalculationThread::readPostMovbsTxt(QString file_path) {
	QFileInfo file_info(file_path);
	if (!file_info.isFile()) return;
	FILE* f_movbs = fopen(file_path.toLocal8Bit().data(), "r");
	if (f_movbs) {
		char line[256] = { 0 };
		int line_num = 0;
		int time_index = 0;
		double sum_heading = 0.0;
		double sum_movbs_length = 0.0;
		int heading_num = 0;
		while (fgets(line, 256, f_movbs) != NULL) {
			line_num++;
			if (line_num == 1)
				continue;
			QString q_line(line);
			QStringList items = q_line.split(",");
			if (items.size() < 21)
				continue;
			uint32_t time = (uint32_t)items[1].trimmed().toDouble();
			if (time < m_TimeSlice[time_index].starttime)
				continue;
			if (time > m_TimeSlice[time_index].endtime) {//区间结束进行统计
				if (time_index < angle_list.size()) {
					if (heading_num > 0) { //统计平均heading
						angle_list[time_index].movbs_heading = sum_heading / heading_num;
						if (angle_list[time_index].movbs_heading < 0) {
							angle_list[time_index].movbs_heading += 360;
						}
						angle_list[time_index].movbs_length = sum_movbs_length / heading_num;
					}
				}
				time_index++;//递增区间id
				sum_heading = 0.0;
				sum_movbs_length = 0.0;
				heading_num = 0;
				if (time_index >= m_TimeSlice.size())//最后一个区间计算完成
					break;
			}
			if (time >= m_TimeSlice[time_index].starttime && time < m_TimeSlice[time_index].endtime) {//累计区间内的heading
				int type1 = items[2].trimmed().toInt();
				int type2 = items[3].trimmed().toInt();
				if (type1 == 4 && type2 == 4) {
					double heading = items[19].trimmed().toDouble();
					double north_len = items[16].trimmed().toDouble();
					double east_len = items[17].trimmed().toDouble();
					double up_len = items[18].trimmed().toDouble();
					if (heading < 0) {
						heading += 360;
					}
					sum_heading += heading;
					double movbs_length = sqrt(north_len * north_len + east_len * east_len + up_len * up_len);
					sum_movbs_length += movbs_length;
					heading_num++;
				}
			}
		}
	}
}

void CalculationThread::CalculateAverageAngle()
{
	for (int i = 0; i < angle_list.size(); i++) {
		angle_list[i].roll = angle_list[i].ins_roll;
		angle_list[i].ins_gnss_diff = angle_list[i].ins_heading - angle_list[i].gnss_heading;
		if (angle_list[i].movbs_heading != 0.0) {
			angle_list[i].gnssposvel_movbs_diff = angle_list[i].gnssposvel_heading - angle_list[i].movbs_heading;
		}
	}
	stAngle sum_angle = { 0 };
	memset(&m_avg_angle, 0, sizeof(stAngle));
	int num_in_cal = 0;
	int num_in_cal_roll = 0;
	int num_in_gnssposvel_movbs = 0;
	for (int i = 0; i < angle_list.size(); i++) {
		if (fabs(angle_list[i].ins_gnss_diff - angle_list[i].heading) > 1.0) continue;
		sum_angle.heading += angle_list[i].heading;
		sum_angle.pitch += angle_list[i].pitch;
		num_in_cal++;
	}
	for (int i = 0; i < angle_list.size(); i++) {
		if (m_TimeSlice[i].used_in_roll == 0 || m_TimeSlice[i].distance > 300) continue;
		sum_angle.roll += angle_list[i].roll;
		num_in_cal_roll++;
	}
	for (int i = 0; i < angle_list.size(); i++) {
		if (angle_list[i].gnssposvel_movbs_diff == 0.0) continue;
		if (fabs(angle_list[i].gnssposvel_movbs_diff) >= 5.0) continue;
		sum_angle.gnssposvel_movbs_diff += angle_list[i].gnssposvel_movbs_diff;
		sum_angle.movbs_length += angle_list[i].movbs_length;
		num_in_gnssposvel_movbs++;
	}
	if (num_in_cal > 0) {
		m_avg_angle.heading = sum_angle.heading / num_in_cal;
		m_avg_angle.pitch = sum_angle.pitch / num_in_cal;
	}
	if (num_in_cal_roll > 0) {
		m_avg_angle.roll = sum_angle.roll / num_in_cal_roll;
	}
	if (num_in_gnssposvel_movbs > 0) {
		m_avg_angle.gnssposvel_movbs_diff = sum_angle.gnssposvel_movbs_diff / num_in_gnssposvel_movbs;
		m_avg_angle.movbs_length = sum_angle.movbs_length / num_in_gnssposvel_movbs;
	}
	WriteAngleFile();
	WriteAverageAngleFile();
}

void CalculationThread::WriteAngleFile()
{
	QFileInfo result_file(m_PostInsFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.completeBaseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();
	QString file_name = absoluteDir + QDir::separator() + basename + "_Angle.txt";
	QFile file(file_name);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		file.write(QString::asprintf(
			"%4s,%6s,%6s,"
			"%6s,%8s,%8s,%4s,"
			"%8s,%8s,"
			"%7s,%7s,%7s,"
			"%8s,%8s,%11s,%7s,"
			"%9s,%9s,%9s,%15s\n",
			"week", "start", "end",
			"during", "dis", "dis_s", "turn",
			"dst_s", "dst_e",
			"roll", "pitch", "heading",
			"gnss_hd", "ins_hd", "ins-gnss_hd", "diff_hd",
			"posvel_hd", "movbs_hd", "movbs_len", "posvel-movbs_hd").toLocal8Bit());
		if (m_TimeSlice.size() == angle_list.size()) {
			for (int i = 0; i < angle_list.size(); i++) {
				file.write(QString::asprintf(
					"%4d,%6d,%6d,"
					"%6d,%8.2f,%8.2f,%4d,"
					"%8.2f,%8.2f,"
					"%7.3f,%7.3f,%7.3f,"
					"%8.3f,%8.3f,%11.3f,%7.3f,"
					"%9.3f,%9.3f,%9.3f,%15.3f\n",
					m_TimeSlice[i].week, m_TimeSlice[i].starttime / 1000, m_TimeSlice[i].endtime / 1000,
					m_TimeSlice[i].during / 1000, m_TimeSlice[i].distance, m_TimeSlice[i].speed_distance, m_TimeSlice[i].used_in_roll,
					m_TimeSlice[i].dst_s, m_TimeSlice[i].dst_e,
					angle_list[i].roll, angle_list[i].pitch, angle_list[i].heading,
					angle_list[i].gnss_heading, angle_list[i].ins_heading, angle_list[i].ins_gnss_diff, (angle_list[i].heading - angle_list[i].ins_gnss_diff),
					angle_list[i].gnssposvel_heading, angle_list[i].movbs_heading, angle_list[i].movbs_length, angle_list[i].gnssposvel_movbs_diff).toLocal8Bit());
			}
		}
		file.close();
	}
}

void CalculationThread::WriteAverageAngleFile() {
	QFileInfo result_file(m_PostInsFilePath);
	if (!result_file.isFile()) return;
	QString basename = result_file.completeBaseName();
	QString absoluteDir = result_file.absoluteDir().absolutePath();
	QString file_name = absoluteDir + QDir::separator() + basename + "_Angle_Result.txt";
	QFile file(file_name);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		file.write(QString::asprintf("%7.3f,%7.3f,%7.3f,%7.3f,%7.3f",
			m_avg_angle.roll, m_avg_angle.pitch, m_avg_angle.heading, m_avg_angle.gnssposvel_movbs_diff, m_avg_angle.movbs_length).toLocal8Bit());
		file.close();
	}
}
#include "MergeThread.h"
#include <QDir>
#include <QDebug>

MergeThread::MergeThread(QObject *parent)
	: QThread(parent)
	, m_isStop(true)
	, m_MergeFormat(emMergeFromat_rtcm_imu)
	, m_outlen(0)
	, m_rtcmFile(NULL)
	, m_imuFile(NULL)
	, m_roverFile(NULL)
	, m_rover2File(NULL)
	, m_baseFile(NULL)
	, m_outFile(NULL)
	, m_logFile(NULL)
	, m_RawFilesSize(0)
	, m_RawFilesReadSize(0)
	, last_TimeOfWeek(0)
	, per10Hz(0)
	, last_per10Hz(0)
{
	memset(m_outbuff, 0, MAX_BUFFER_SIZE);
	memset(&m_gnss, 0, sizeof(m_gnss));
	memset(&m_gnss2, 0, sizeof(m_gnss2));
}

MergeThread::~MergeThread()
{
}

void MergeThread::run()
{
	m_isStop = false;
	if (!m_MergeFileName1.isEmpty() && !m_MergeFileName2.isEmpty()) {
		m_TimeCounter.start();
		m_RawFilesReadSize = 0;
		makeOutFileName();
		switch (m_MergeFormat)
		{
		case emMergeFromat_rtcm_imu:
			mergeRtcmImuFile();
			break;
		case emMergeFromat_rover_base:
			mergeRoverBaseFile();
			break;
		case emMergeFromat_ins_csv_imu_txt:
			merge_process_txt_imu_txt_realtime();
			break;
		case emMergeFromat_process_txt_imu_txt:
			merge_process_txt_imu_txt_post();
			break;
		case emMergeFromat_process_gnss_csv:
			merge_process_gnss_csv();
			break;
		default:
			break;
		}
	}
	emit sgnFinished();
}

void MergeThread::stop()
{
	m_isStop = true;
}

void MergeThread::setMergeFormat(int format)
{
	m_MergeFormat = format;
}

void MergeThread::setMergeFileName1(QString file)
{
	m_MergeFileName1 = file;
}

void MergeThread::setMergeFileName2(QString file)
{
	m_MergeFileName2 = file;
}

void MergeThread::setMergeFileName3(QString file)
{
	m_MergeFileName3 = file;
}

void MergeThread::makeOutFileName()
{
	QFileInfo outDir(m_MergeFileName1);
	if (outDir.isFile()) {
		QString basename = outDir.completeBaseName();
		QString absoluteDir = outDir.absoluteDir().absolutePath();
		switch (m_MergeFormat)
		{
			case emMergeFromat_rtcm_imu:
			{
				m_OutFileName = absoluteDir + QDir::separator() + basename + ".imu-rtcm";
				m_OutLogName = absoluteDir + QDir::separator() + basename + ".imu-rtcm.log";
			}
			break;
			case emMergeFromat_rover_base:
			{
				m_OutFileName = absoluteDir + QDir::separator() + basename + ".rover-base";
				m_OutLogName = absoluteDir + QDir::separator() + basename + ".rover-base.log";
			}
			break;
			case emMergeFromat_process_txt_imu_txt:
			{
				m_OutFileName = absoluteDir + QDir::separator() + basename + "_mixed.csv";
				m_OutLogName = absoluteDir + QDir::separator() + basename + "_mixed.log";
			}
			break;
		}
	}
}

void MergeThread::mergeRtcmImuFile() {
	if (is_aceinna_decoding()) return;
	set_aceinna_decoding(1);
	m_rtcmFile = fopen(m_MergeFileName1.toLocal8Bit().data(),"rb");
	m_imuFile = fopen(m_MergeFileName2.toLocal8Bit().data(), "rb");
	m_outFile = fopen(m_OutFileName.toLocal8Bit().data(), "wb");
	m_logFile = fopen(m_OutLogName.toLocal8Bit().data(), "w");
	if (m_rtcmFile && m_imuFile && m_outFile && m_logFile) {
		m_RawFilesSize = getFileSize(m_rtcmFile);
		m_RawFilesSize += getFileSize(m_imuFile);
		m_roverbuff.clear();
		m_basebuff.clear();
		m_imubuff.clear();
		memset(&m_gnss, 0, sizeof(m_gnss));
		int stn = 0;
		per10Hz = 0;
		last_per10Hz = 0;
		while (!feof(m_rtcmFile) && !feof(m_imuFile)) {
			if (m_isStop) break;
			int ret = readOneImu();
			imu_t* pak = getImuPak();
			per10Hz = pak->gps_millisecs / 100;
			if (ret = 1 && per10Hz != last_per10Hz) {
				last_per10Hz = per10Hz;
				while (!feof(m_rtcmFile)) {
					if (m_isStop) break;
					if(TYPE_ROV == stn && m_roverbuff.size() > 0){
						//last rtcm is later
					}
					else {
						stn = readOneRtcm();
					}
					if (TYPE_ROV == stn) {
						int week = 0;
						double second = time2gpst(m_gnss.obs[0].time, &week);
						uint32_t nsec = second * 1000;
						if (week == pak->GPS_Week) {
							if (last_TimeOfWeek <= nsec && nsec <= pak->gps_millisecs) {
								fwrite(m_roverbuff.data(), 1, m_roverbuff.size(), m_outFile);
								fprintf(m_logFile, "rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
								m_roverbuff.clear();
								//break;
							}
							else if (nsec > pak->gps_millisecs) {
								//fprintf(m_logFile, "later rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
								break;
							}
							else {
								fprintf(m_logFile, "early rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
								m_roverbuff.clear();
							}
						}
						else {
							fprintf(m_logFile, "week error rover,week:%d,sec:%d,size:%d\n", week, nsec, m_roverbuff.size());
							m_roverbuff.clear();
						}
					}
					else if (TYPE_BAS == stn) {
						int week = 0;
						double second = time2gpst(m_gnss.obs[1].time, &week);
						uint32_t nsec = second * 1000;
						if (m_gnss.obs[0].time.time - 5 < m_gnss.obs[1].time.time && m_gnss.obs[1].time.time < m_gnss.obs[0].time.time + 5) {
							fwrite(m_basebuff.data(), 1, m_basebuff.size(), m_outFile);
							fprintf(m_logFile, "base,week:%d,sec:%d,size:%d\n", week, nsec, m_basebuff.size());
						}
						m_basebuff.clear();
					}
				}
			}
		}
	}
	if (m_rtcmFile) fclose(m_rtcmFile); m_rtcmFile = NULL;
	if (m_imuFile) fclose(m_imuFile); m_imuFile = NULL;
	if (m_outFile) fclose(m_outFile); m_outFile = NULL;
	if (m_logFile) fclose(m_logFile); m_logFile = NULL;
	set_aceinna_decoding(0);
}

void MergeThread::mergeRoverBaseFile()
{
	m_roverFile = fopen(m_MergeFileName1.toLocal8Bit().data(), "rb");
	m_baseFile = fopen(m_MergeFileName2.toLocal8Bit().data(), "rb");
	m_outFile = fopen(m_OutFileName.toLocal8Bit().data(), "wb");
	m_logFile = fopen(m_OutLogName.toLocal8Bit().data(), "w");
	if (!m_MergeFileName3.isEmpty()) {
		m_rover2File = fopen(m_MergeFileName3.toLocal8Bit().data(), "rb");
	}
	if (m_roverFile && m_baseFile && m_outFile && m_logFile) {
		m_RawFilesSize = getFileSize(m_roverFile);
		m_RawFilesSize += getFileSize(m_baseFile);
		if (m_rover2File) {
			m_RawFilesSize += getFileSize(m_rover2File);
		}
		m_roverbuff.clear();
		m_basebuff.clear();
		m_rover2buff.clear();
		memset(&m_gnss, 0, sizeof(m_gnss));
		memset(&m_gnss2, 0, sizeof(m_gnss2));
		while (!feof(m_roverFile) && !feof(m_baseFile)) {
			readOneRover();
			if (m_rover2File) {
				if (feof(m_rover2File)) break;
				readOneRover2();
			}
			readOneBase();
		}
	}
	if (m_roverFile) fclose(m_roverFile); m_roverFile = NULL;
	if (m_rover2File) fclose(m_rover2File); m_rover2File = NULL;
	if (m_baseFile) fclose(m_baseFile); m_baseFile = NULL;
	if (m_outFile) fclose(m_outFile); m_outFile = NULL;
	if (m_logFile) fclose(m_logFile); m_logFile = NULL;
}

void MergeThread::merge_process_txt_imu_txt_post()
{
	QFile m_process_txt(m_MergeFileName1);
	QFile m_imu_txt(m_MergeFileName2);
	QFile m_gnss_txt(m_MergeFileName3);
	QFile m_ins_txt(m_MergeFileName1+"_ins.txt");
	QFile m_mixed_csv(m_MergeFileName1 + "_mixed.csv");

	if (!m_process_txt.open(QIODevice::ReadOnly| QIODevice::Text))
		return;
	if (!m_imu_txt.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	if (!m_gnss_txt.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	if (!m_ins_txt.open(QIODevice::WriteOnly | QIODevice::Text))
		return;
	if (!m_mixed_csv.open(QIODevice::WriteOnly | QIODevice::Text))
		return;

	QMap<int, int> gnss_map;
	while (m_gnss_txt.atEnd() == false) {
		QByteArray buf = m_gnss_txt.readLine();
		QString line(buf);
		QStringList items = line.split(',');
		if (items.size() >= 11) {
			int time = int(items[1].toFloat());
			int satellite = items[9].toInt();
			gnss_map.insert(time, satellite);
		}
	}

	QString title = QString::fromLocal8Bit("GPS周内秒,经度(°),纬度(°),高度(m),向北速度(m/s),向东速度(m/s),向地速度(m/s),航向角(°),俯仰角(°),翻滚角(°),加表x(m/s^2),加表y(m/s^2),加表z(m/s^2),陀螺x(°/s),陀螺y(°/s),陀螺z(°/s),GPS定位状态,GPS定向状态,GPS卫星数量\n");
	m_mixed_csv.write(title.toLocal8Bit());
	m_RawFilesSize = m_process_txt.size();
	while (m_process_txt.atEnd() == false) {
		QByteArray buf = m_process_txt.readLine();
		UpdateProcess(buf.size());
		QString line(buf);
		QStringList items = line.split(',');
		if (items.size() == 23) {
			QStringList new_items;
			for (int i = 0; i < 11; i++) {
				new_items << items[i];
			}
			new_items << items[20];
			new_items << items[19];
			QString new_line = new_items.join(',');
			new_line += "\n";
			m_ins_txt.write(new_line.toLocal8Bit());

			double ins_week_second = items[1].toDouble();
			uint32_t ins_week_second_10 = uint32_t(ins_week_second * 100);
			while (m_imu_txt.atEnd() == false) {
				QByteArray imu_buf = m_imu_txt.readLine();
				QString imu_line(imu_buf);
				QStringList imu_items = imu_line.split(',');
				if (imu_items.size() >= 9) {
					double imu_week_second = imu_items[1].toDouble();
					uint32_t imu_week_second_10 = uint32_t(imu_week_second * 100);
					if (ins_week_second_10 == imu_week_second_10) {
						QStringList mixed_items;
						mixed_items << items[1];
						mixed_items << items[3];
						mixed_items << items[2];
						for (int i = 4; i < 8; i++) {
							mixed_items << items[i];
						}
						//double vel_up = items[7].toDouble();
						//double vel_down = -vel_up;
						//mixed_items << QString::asprintf("%8.5f", vel_down);
						double heading = items[10].toDouble();
						if (heading > 180) {
							heading -= 360;
						}
						mixed_items << QString::asprintf("%10.5f", heading);
						mixed_items << items[9];
						mixed_items << items[8];
						for (int i = 3; i < 9; i++) {
							if (i == 8) {
								imu_items[i].chop(1);
								mixed_items << imu_items[i];
							}
							else {
								mixed_items << imu_items[i];
							}							
						}
						mixed_items << items[20];
						mixed_items << items[19];

						QMap<int, int>::iterator it = gnss_map.find(int(ins_week_second));
						if (it != gnss_map.end()) {
							mixed_items << QString::asprintf("%3d",it.value());
						}
						else {
							mixed_items << QString::asprintf("%3d", 0);
						}

						QString mixed_line = mixed_items.join(',');
						mixed_line += "\n";
						m_mixed_csv.write(mixed_line.toLocal8Bit());
						break;
					}
					else if(imu_week_second > ins_week_second){
						break;
					}
				}
			}
		}
		
	}
	m_process_txt.close();
	m_imu_txt.close();
	m_ins_txt.close();
	m_mixed_csv.close();
}

void MergeThread::merge_process_txt_imu_txt_realtime()
{
	QFile m_process_txt(m_MergeFileName1);
	QFile m_imu_txt(m_MergeFileName2);
	QFile m_gnss_txt(m_MergeFileName3);
	QFile m_mixed_csv(m_MergeFileName1 + "_mixed.csv");

	if (!m_process_txt.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	if (!m_imu_txt.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	if (!m_gnss_txt.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	if (!m_mixed_csv.open(QIODevice::WriteOnly | QIODevice::Text))
		return;

	QMap<int, int> gnss_map;
	while (m_gnss_txt.atEnd() == false) {
		QByteArray buf = m_gnss_txt.readLine();
		QString line(buf);
		QStringList items = line.split(',');
		if (items.size() >= 11) {
			int time = int(items[1].toFloat());
			int satellite = items[9].toInt();
			gnss_map.insert(time, satellite);
		}
	}

	QString title = QString::fromLocal8Bit("GPS周内秒,经度(°),纬度(°),高度(m),向北速度(m/s),向东速度(m/s),向地速度(m/s),航向角(°),俯仰角(°),翻滚角(°),加表x(m/s^2),加表y(m/s^2),加表z(m/s^2),陀螺x(°/s),陀螺y(°/s),陀螺z(°/s),GPS定位状态,GPS定向状态,GPS卫星数量\n");
	m_mixed_csv.write(title.toLocal8Bit());
	m_RawFilesSize = m_process_txt.size();
	int index = 0;
	while (m_process_txt.atEnd() == false) {
		QByteArray buf = m_process_txt.readLine();
		UpdateProcess(buf.size());
		index++;
		if (index == 1) continue;
		QString line(buf);
		QStringList items = line.split(',');
		if (items.size() >= 26) {

			double ins_week_second = items[1].toDouble();
			uint32_t ins_week_second_10 = uint32_t(ins_week_second * 100);
			while (m_imu_txt.atEnd() == false) {
				QByteArray imu_buf = m_imu_txt.readLine();
				QString imu_line(imu_buf);
				QStringList imu_items = imu_line.split(',');
				if (imu_items.size() >= 9) {
					double imu_week_second = imu_items[1].toDouble();
					uint32_t imu_week_second_10 = uint32_t(imu_week_second * 100);
					if (ins_week_second_10 == imu_week_second_10) {
						QStringList mixed_items;
						mixed_items << items[1];
						mixed_items << items[5];
						mixed_items << items[4];
						for (int i = 6; i < 9; i++) {
							mixed_items << items[i];
						}
						double vel_up = items[9].toDouble();
						double vel_down = -vel_up;
						mixed_items << QString::asprintf("%8.5f", vel_down);

						double heading = items[14].toDouble();
						if (heading > 180) {
							heading -= 360;
						}
						mixed_items << QString::asprintf("%10.5f", heading);
						mixed_items << items[13];
						mixed_items << items[12];
						for (int i = 3; i < 9; i++) {
							if (i == 8) {
								imu_items[i].chop(1);
								mixed_items << imu_items[i];
							}
							else {
								mixed_items << imu_items[i];
							}
						}
						mixed_items << items[3];
						mixed_items << items[2];

						QMap<int, int>::iterator it = gnss_map.find(int(ins_week_second));
						if (it != gnss_map.end()) {
							mixed_items << QString::asprintf("%3d", it.value());
						}
						else {
							mixed_items << QString::asprintf("%3d", 0);
						}

						QString mixed_line = mixed_items.join(',');
						mixed_line += "\n";
						m_mixed_csv.write(mixed_line.toLocal8Bit());
						break;
					}
					else if (imu_week_second > ins_week_second) {
						break;
					}
				}
			}
		}

	}
	m_process_txt.close();
	m_imu_txt.close();
	m_gnss_txt.close();
	m_mixed_csv.close();
}

void MergeThread::replace_gnss_line(QStringList& process_item, QStringList& gnss_item) {
	process_item[3] = gnss_item[3];
	process_item[4] = gnss_item[4];
	process_item[5] = gnss_item[5];
	process_item[6] = gnss_item[6];
	process_item[7] = gnss_item[7];
	process_item[8] = gnss_item[8];
	process_item[9] = gnss_item[2];
}

void MergeThread::replace_vel_line(QStringList& process_item, QStringList& gnss_item) {
	float north_vel = gnss_item[13].toFloat();
	float east_vel = gnss_item[14].toFloat();
	float up_vel = gnss_item[15].toFloat();
	process_item[3] = QString::asprintf("%0.4f", sqrt(pow(north_vel, 2) + pow(east_vel, 2)));
	process_item[4] = QString::asprintf("%0.4f", atan2(east_vel, north_vel) * 57.295779513082320);
	process_item[5] = QString::asprintf("%0.4f", up_vel);
}

void MergeThread::merge_process_gnss_csv()
{
	QFile m_process(m_MergeFileName1);
	QFile m_gnss_csv(m_MergeFileName2);
	QFile m_process_new(m_MergeFileName1+"-new");

	if (!m_process.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	if (!m_gnss_csv.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	if (!m_process_new.open(QIODevice::WriteOnly | QIODevice::Text))
		return;

	int process_start = 0; //开始后插入gnss以外的数据
	while (m_gnss_csv.atEnd() == false) {
		QByteArray buf = m_gnss_csv.readLine();
		QString gnss_line(buf);
		QStringList gnss_items = gnss_line.trimmed().split(',');
		if (gnss_items.size() >= 13) {
			int week = gnss_items[0].toInt();
			float second = gnss_items[1].toFloat();
			while (m_process.atEnd() == false) {
				QByteArray buf2 = m_process.readLine();
				QString process_line(buf2);
				if (process_line.startsWith("$GPGNSS")) {
					QStringList process_item = process_line.trimmed().split(',');
					if (process_item.size() >= 10) {
						int week2 = process_item[1].toInt();
						float second2 = process_item[2].toFloat();
						if (week != week2) {
							break;
						}else if(second < second2){
                            break;
                        }
						else if (fabs(second - second2) < 0.1) {
							process_start = 1;
							replace_gnss_line(process_item, gnss_items);
							QString new_process_lines = process_item.join(',');
							new_process_lines += "\n";
							m_process_new.write(new_process_lines.toLocal8Bit());
						}
					}
				}
				else if (process_line.startsWith("$GPVEL")) {
					QStringList process_item = process_line.trimmed().split(',');
					if (process_item.size() >= 5) {
						int week2 = process_item[1].toInt();
						float second2 = process_item[2].toFloat();
						if (week != week2) {
							break;
						}
						else if (second < second2) {
							break;
						}
						else if (fabs(second - second2) < 0.1) {
							process_start = 1;
							replace_vel_line(process_item, gnss_items);
							QString new_process_lines = process_item.join(',');
							new_process_lines += "\n";
							m_process_new.write(new_process_lines.toLocal8Bit());
						}
					}
				}
				else {
					if (process_start == 1) {
						m_process_new.write(buf2);
					}
				}
			}
		}
	}

	m_process.close();
	m_gnss_csv.close();
	m_process_new.close();
}

int MergeThread::readOneRtcm() {
	char c = 0;
	int stn = 0;
	bool is_complete = false;
	while (!feof(m_rtcmFile)) {
		if (m_isStop) break;
		fread(&c, sizeof(char), 1, m_rtcmFile);
		stn = input_aceinna_format_raw(c, m_outbuff, &m_outlen);
		if (TYPE_ROV == stn) {
			m_roverbuff.append((char*)m_outbuff, m_outlen);
			UpdateProcess(m_outlen);
		}
		else if (TYPE_BAS == stn) {
			m_basebuff.append((char*)m_outbuff, m_outlen);
			UpdateProcess(m_outlen);
		}
		if (TYPE_ROV == stn || TYPE_BAS == stn) {
			for (int j = 0; j < m_outlen; j++) {
				int ret = input_rtcm3(m_outbuff[j], stn - 1, &m_gnss);
				if (ret == 1) {
					is_complete = true;
					break;
				}
			}
		}
		if (is_complete) {
			break;
		}
	}
	return stn;
}

int MergeThread::readOneImu()
{
	char c = 0;
	char out_msg[1024] = { 0 };
	char imu_buffer[64] = { 0 };
	int ret = 0;
	if (m_imubuff.size() > 0) {
		fwrite(m_imubuff.data(), 1, m_imubuff.size(), m_outFile);
		m_imubuff.clear();
	}
	while (!feof(m_imuFile)) {
		if (m_isStop) break;
		fread(&c, sizeof(char), 1, m_imuFile);
		ret = input_imu_raw(c, out_msg);
		if (1 == ret) {
			imu_t* pak = getImuPak();
			if (pak->GPS_Week == 0) {
				m_imubuff.clear();
				continue;
			}
			if (pak->gps_millisecs % 100 == 0 || (pak->gps_millisecs / 10 > last_TimeOfWeek/10 && last_TimeOfWeek > 0)) {
				sprintf(imu_buffer, IMU_FLAG);
				memcpy(imu_buffer + ACEINNA_HEAD_SIZE, pak, sizeof(imu_t));
				m_imubuff.append(imu_buffer, IMU_CONST_SIZE + ACEINNA_HEAD_SIZE);
				UpdateProcess(ACEINNA_HEAD_SIZE + IMU_CONST_SIZE+2);
				fprintf(m_logFile, "$IMU,week:%d,sec:%d,len:%llu\n", pak->GPS_Week, pak->gps_millisecs, sizeof(imu_t));
				break;
			}
			else {
				sprintf(imu_buffer, IMU_FLAG);
				memcpy(imu_buffer + ACEINNA_HEAD_SIZE, pak, sizeof(imu_t));
				fwrite(imu_buffer, 1, IMU_CONST_SIZE + ACEINNA_HEAD_SIZE, m_outFile);
				UpdateProcess(ACEINNA_HEAD_SIZE + IMU_CONST_SIZE + 2);
				fprintf(m_logFile, "$IMU,week:%d,sec:%d,len:%llu\n", pak->GPS_Week, pak->gps_millisecs, sizeof(imu_t));
			}
			last_TimeOfWeek = pak->gps_millisecs;
		}
	}
	return ret;
}

void MergeThread::readOneRover()
{
	char c = 0;
	int ret = 0;
	uint8_t* buffer = NULL;
	int len = 0;
	while (!feof(m_roverFile)) {
		fread(&c, sizeof(char), 1, m_roverFile);
		m_roverbuff.append(c);
		ret = input_rtcm3(c, 0, &m_gnss);
		if (is_complete_rtcm()) {
			UpdateProcess(m_roverbuff.size());
			if(m_gnss.obs[0].time.time > 0){
				fprintf(m_logFile, "%s1,%03d,%f\n", ROV_FLAG, m_roverbuff.size(), m_gnss.obs[0].time.time + m_gnss.obs[0].time.sec);
				fprintf(m_outFile, "%s1%03d", ROV_FLAG, m_roverbuff.size());
				fwrite(m_roverbuff.data(), 1, m_roverbuff.size(), m_outFile);
			}
			//buffer = (uint8_t*)m_roverbuff.data();
			//len = m_roverbuff.size() - 3;
			//fprintf(m_logFile, "%s,1,%03d, %d\n", ROV_FLAG, m_roverbuff.size(),rtcm_getbitu(buffer, len * 8, 24));
			m_roverbuff.clear();
		}
		if (ret == 1) {
			int week = 0;
			double second = time2gpst(m_gnss.obs[0].time, &week);
			uint32_t nsec = second * 1000;
			fprintf(m_logFile, "ROV1,%f,week:%d,sec:%d\n", m_gnss.obs[0].time.time + m_gnss.obs[0].time.sec, week, nsec);
			break;
		}
	}
}

void MergeThread::readOneRover2()
{
	char c = 0;
	int ret = 0;
	uint8_t* buffer = NULL;
	int len = 0;
	if (m_gnss2.obs[0].time.time + m_gnss2.obs[0].time.sec >= m_gnss.obs[0].time.time + m_gnss.obs[0].time.sec) {
		return;
	}
	while (!feof(m_rover2File)) {
		fread(&c, sizeof(char), 1, m_rover2File);
		m_rover2buff.append(c);
		ret = input_rtcm3(c, 0, &m_gnss2);
		if (is_complete_rtcm()) {
			UpdateProcess(m_rover2buff.size());
			if (m_gnss2.obs[0].time.time > 0) {
				fprintf(m_logFile, "%s2,%03d,%f\n", ROV_FLAG, m_rover2buff.size(), m_gnss2.obs[0].time.time + m_gnss2.obs[0].time.sec);
				fprintf(m_outFile, "%s2%03d", ROV_FLAG, m_rover2buff.size());
				fwrite(m_rover2buff.data(), 1, m_rover2buff.size(), m_outFile);
			}
			m_rover2buff.clear();
			if (m_gnss2.obs[0].time.time + m_gnss2.obs[0].time.sec < m_gnss.obs[0].time.time + m_gnss.obs[0].time.sec) {
				continue;
			}
		}
		if (ret == 1) {
			int week = 0;
			double second = time2gpst(m_gnss2.obs[0].time, &week);
			uint32_t nsec = second * 1000;
			fprintf(m_logFile, "ROV2,%f,week:%d,sec:%d\n", m_gnss2.obs[0].time.time + m_gnss2.obs[0].time.sec, week, nsec);
			break;
		}
	}
}

void MergeThread::readOneBase()
{
	char c = 0;
	int ret = 0;
	uint8_t* buffer = NULL;
	int len = 0;
	char flag[16] = { 0 };
	if (m_gnss.obs[1].time.time >= m_gnss.obs[0].time.time) return;
	QByteArray writeBuffer;
	while (!feof(m_baseFile)) {
		fread(&c, sizeof(char), 1, m_baseFile);
		m_basebuff.append(c);
		ret = input_rtcm3(c, 1, &m_gnss);
		if (is_complete_rtcm()) {
			UpdateProcess(m_basebuff.size());
			sprintf(flag, "%s%03d", BAS_FLAG, m_basebuff.size());
			writeBuffer.append(flag);
			writeBuffer.append(m_basebuff);
			//fprintf(m_logFile, "%s,0,%03d,%03d\n", BAS_FLAG, m_basebuff.size(), m_basebuff.size() + 8);
			//buffer = (uint8_t*)m_basebuff.data();
			//len = m_basebuff.size() - 3;
			//fprintf(m_logFile, "%s,0,%03d, %d\n", BAS_FLAG, m_basebuff.size(), rtcm_getbitu(buffer, len * 8, 24));
			m_basebuff.clear();
		}
		if (ret == 1) {
			if (m_gnss.obs[1].time.time >= m_gnss.obs[0].time.time) {
				int week = 0;
				double second = time2gpst(m_gnss.obs[0].time, &week);
				uint32_t nsec = second * 1000;
				fprintf(m_logFile, "BAS,%f,week:%d,sec:%d\n", m_gnss.obs[1].time.time + m_gnss.obs[1].time.sec, week, nsec);
				fwrite(writeBuffer.data(), 1, writeBuffer.size(), m_outFile);
				break;
			}
			writeBuffer.clear();
		}
	}
}

void MergeThread::UpdateProcess(int size) {
	m_RawFilesReadSize += size;
	double percent = (double)m_RawFilesReadSize / (double)m_RawFilesSize * 10000;
	emit sgnProgress((int)percent, m_TimeCounter.elapsed());
	//qDebug("%d/%d", m_RawFilesReadSize, m_RawFilesSize);
}

int MergeThread::getFileSize(FILE * file)
{
	fseek(file, 0L, SEEK_END);
	int file_size = ftell(file);
	fseek(file, 0L, SEEK_SET);
	return file_size;
}
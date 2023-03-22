#include "CsvAnalysisThread.h"
#include <QFile>
#include <QDir>
CsvAnalysisThread::CsvAnalysisThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, m_FileFormat(emAnalysisPostGnssCsv)
{
	m_SkipLine = 1;
	start_column = 1;
	m_StaticAnalysis = new StaticAnalysis(this);
	m_ins_kml = new Kml_Ins_Obj();
}

CsvAnalysisThread::~CsvAnalysisThread()
{
	delete m_StaticAnalysis;
}

void CsvAnalysisThread::run()
{
	m_isStop = false;
	m_TimeCounter.start();
	if (!m_FileName.isEmpty()) {
		switch (m_FileFormat)
		{
		case emAnalysisPostGnssCsv:
			AnalysisPostGnssCsv();
			break;
		case emPostInsCsv2Kml:
			PostInsCsv2Kml();
			break;
		default:
			break;
		}
	}
	emit sgnFinished();
}

void CsvAnalysisThread::stop()
{
	m_isStop = true;
}

void CsvAnalysisThread::setFileFormat(int format)
{
	m_FileFormat = format;
}

void CsvAnalysisThread::setFileName(QString file)
{
	m_FileName = file;
}

QString CsvAnalysisThread::getBasePath() {
	QFileInfo fileinfo(m_FileName);
	QString basename = fileinfo.completeBaseName();
	QString absoluteDir = fileinfo.absoluteDir().absolutePath();
	return absoluteDir + QDir::separator() + basename;
}

void CsvAnalysisThread::AnalysisGnssCsv()
{
	if (m_FileName.isEmpty()) return;
	if (!QFile::exists(m_FileName))return;
	QFile CsvFile(m_FileName);
	if (CsvFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
		int32_t line_num = 0;
		int64_t file_size = CsvFile.size();
		int64_t read_size = 0;
		m_StaticAnalysis->init();
		m_StaticAnalysis->set_out_base_name(getBasePath());
		while (!CsvFile.atEnd() && !m_isStop) {
			QByteArray byte_line = CsvFile.readLine(512);
			read_size = CsvFile.pos();
			line_num++;
			if (line_num <= m_SkipLine) continue;
			QByteArrayList byte_items = byte_line.trimmed().split(',');
			if (byte_items.size() >= 16) {
				QList<double> value_list;
				for (int i = 0; i < byte_items.size(); i++) {
					value_list.append(byte_items[i].trimmed().toDouble());
				}
				static_gnss_t gnss = { 0 };
				gnss.gps_week = uint16_t(value_list[start_column + 0]);
				gnss.gps_millisecs = uint32_t(value_list[start_column + 1] * 1000);
				gnss.position_type = uint8_t(value_list[start_column + 2]);
				gnss.latitude = value_list[start_column + 3];
				gnss.longitude = value_list[start_column + 4];
				gnss.height = value_list[start_column + 5];
				gnss.north_vel = value_list[start_column + 13];
				gnss.east_vel = value_list[start_column + 14];
				gnss.up_vel = value_list[start_column + 15];
				m_StaticAnalysis->append_gnss_sol(&gnss);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		m_StaticAnalysis->gnss_summary();
		CsvFile.close();
	}
}

void CsvAnalysisThread::AnalysisPostGnssCsv()
{
	if (m_FileName.isEmpty()) return;
	if (!QFile::exists(m_FileName))return;
	QFile CsvFile(m_FileName);
	if (CsvFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
		int32_t line_num = 0;
		int64_t file_size = CsvFile.size();
		int64_t read_size = 0;
		start_column = 0;
		m_StaticAnalysis->init();
		m_StaticAnalysis->set_out_base_name(getBasePath());
		while (!CsvFile.atEnd() && !m_isStop) {
			QByteArray byte_line = CsvFile.readLine(512);
			read_size = CsvFile.pos();
			line_num++;
			if (line_num <= m_SkipLine) continue;
			QByteArrayList byte_items = byte_line.trimmed().split(',');
			if (byte_items.size() >= 16) {
				QList<double> value_list;
				for (int i = 0; i < byte_items.size(); i++) {
					value_list.append(byte_items[i].trimmed().toDouble());
				}
				static_gnss_t gnss = { 0 };
				gnss.gps_week = uint16_t(value_list[start_column + 0]);
				gnss.gps_millisecs = uint32_t(value_list[start_column + 1] * 1000);
				gnss.position_type = uint8_t(value_list[start_column + 2]);
				gnss.latitude = value_list[start_column + 3];
				gnss.longitude = value_list[start_column + 4];
				gnss.height = value_list[start_column + 5];
				gnss.north_vel = value_list[start_column + 14];
				gnss.east_vel = value_list[start_column + 15];
				gnss.up_vel = value_list[start_column + 16];
				m_StaticAnalysis->append_gnss_sol(&gnss);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
		m_StaticAnalysis->gnss_summary();
		CsvFile.close();
	}
}

void CsvAnalysisThread::PostInsCsv2Kml() {
	if (m_FileName.isEmpty()) return;
	if (!QFile::exists(m_FileName))return;
	QFile CsvFile(m_FileName);
	m_ins_kml->init();
	if (CsvFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
		int32_t line_num = 0;
		int64_t file_size = CsvFile.size();
		int64_t read_size = 0;
		while (!CsvFile.atEnd() && !m_isStop) {
			QByteArray byte_line = CsvFile.readLine(512);
			read_size = CsvFile.pos();
			line_num++;
			if (line_num <= m_SkipLine) continue;
			QByteArrayList byte_items = byte_line.trimmed().split(',');
			if (byte_items.size() >= 24) {
				QList<double> value_list;
				for (int i = 0; i < byte_items.size(); i++) {
					value_list.append(byte_items[i].trimmed().toDouble());
				}
				kml_ins_t ins_kml = { 0 };
				ins_kml.gps_week = uint16_t(value_list[0]);
				ins_kml.gps_secs = value_list[1];
				ins_kml.ins_status = (int)value_list[19];
				ins_kml.ins_position_type = (int)value_list[20];
				ins_kml.latitude = value_list[2];
				ins_kml.longitude = value_list[3];
				ins_kml.height = value_list[4];
				ins_kml.north_velocity = value_list[5];
				ins_kml.east_velocity = value_list[6];
				ins_kml.up_velocity = -value_list[7];
				ins_kml.roll = value_list[8];
				ins_kml.pitch = value_list[9];
				ins_kml.heading = value_list[10];
				m_ins_kml->append_ins(ins_kml);
			}
			double percent = (double)read_size / (double)file_size * 10000;
			emit sgnProgress((int)percent, m_TimeCounter.elapsed());
		}
	}
	m_ins_kml->open_file(m_FileName.toLocal8Bit().data(), "ins.kml");
	m_ins_kml->write_file();
	m_ins_kml->close_file();
	CsvFile.close();
}
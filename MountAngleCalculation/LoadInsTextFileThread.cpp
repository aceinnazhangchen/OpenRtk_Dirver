#include "LoadInsTextFileThread.h"
#include <QFile>
#include <QDir>

LoadInsTextFileThread::LoadInsTextFileThread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
{
	m_SplitByTime = new SplitByTime();
}

LoadInsTextFileThread::~LoadInsTextFileThread()
{
	if(m_SplitByTime) delete m_SplitByTime;
}

void LoadInsTextFileThread::init()
{
	m_SplitByTime->init();

}

void LoadInsTextFileThread::run()
{
	m_isStop = false;
	LoadInsText();
	emit sgnFinished();
}

void LoadInsTextFileThread::stop()
{
	m_isStop = true;
}

void LoadInsTextFileThread::setFileName(QString file)
{
	m_InsTextFileName = file;
}

QString LoadInsTextFileThread::getBasePath() {
	QFileInfo fileinfo(m_InsTextFileName);
	QString basename = fileinfo.completeBaseName();
	QString absoluteDir = fileinfo.absoluteDir().absolutePath();
	return absoluteDir + QDir::separator() + basename;
}

std::vector<stTimeSlice>& LoadInsTextFileThread::get_time_slices()
{
	return m_SplitByTime->get_time_slices();
}

void LoadInsTextFileThread::LoadInsText()
{
	if (m_InsTextFileName.isEmpty()) return;
	if (!QFile::exists(m_InsTextFileName))return;
	QFile InsTextFile(m_InsTextFileName);
	if (InsTextFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
		m_SplitByTime->init();
		while (!InsTextFile.atEnd() && !m_isStop) {
			QByteArray byte_line = InsTextFile.readLine(256);
			QByteArrayList byte_items = byte_line.trimmed().split(',');
			if (byte_items.size() == 23) {
				QList<double> value_list;
				for (int i = 0; i < byte_items.size(); i++) {
					value_list.append(byte_items[i].trimmed().toDouble());
				}
				ins_sol_data ins_data = { 0 };
				ins_data.gps_week = (uint16_t)value_list[0];
				ins_data.gps_millisecs = (uint32_t)(value_list[1]*1000);
				ins_data.ins_status = (uint8_t)value_list[19];
				ins_data.ins_position_type = (uint8_t)value_list[20];
				ins_data.latitude = value_list[2];
				ins_data.longitude = value_list[3];
				ins_data.height = value_list[4];
				ins_data.north_velocity = value_list[5];
				ins_data.east_velocity = value_list[6];
				ins_data.up_velocity = value_list[7];
				ins_data.roll = value_list[8];
				ins_data.pitch = value_list[9];
				ins_data.heading = value_list[10];
				m_SplitByTime->input_sol_data(ins_data);
			}
		}
		m_SplitByTime->finish();
		InsTextFile.close();
	}
}

#pragma once

#include <QThread>
#include <QTime>
#include <QList>
#include "ins401.h"

enum emDecodeFormat {
	emDecodeFormat_OpenRTK330LI,
	emDecodeFormat_RTK330LA,
	emDecodeFormat_Ins401,
};

class SimpleDecodeThread : public QThread
{
	Q_OBJECT

public:
	SimpleDecodeThread(QObject *parent);
	~SimpleDecodeThread();
	void run();
	void stop();
	void setFileFormat(int format);
	void setFileName(QString file);
	void setShowTime(bool show);
	void setKmlFrequency(int frequency);
	void setDateTime(QString time);
	QString& getOutBaseName();
protected:
	void makeOutPath(QString filename);
	void decode_openrtk330li();
	void decode_openrtk_inceptio();
	void decode_ins401();
private:
	bool m_isStop;
	int m_FileFormat;
	QString m_FileName;
	QString m_OutBaseName;
	QTime m_TimeCounter;
	Ins401_Tool::Ins401_decoder* ins401_decoder;
	bool m_show_time;
	int ins_kml_frequency;
	QString m_datatime;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};

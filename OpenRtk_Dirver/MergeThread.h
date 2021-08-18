#pragma once

#include <QThread>
#include <QTime>
#include "mixed_raw.h"
#include "rtcm.h"
#include "imu_raw.h"

enum emMergeFromat
{
	emMergeFromat_rover_base,
	emMergeFromat_rtcm_imu,
};

class MergeThread : public QThread
{
	Q_OBJECT

public:
	MergeThread(QObject *parent);
	~MergeThread();
	void run();
	void stop();
	void setMergeFormat(int format);
	void setMergeFileName1(QString file);
	void setMergeFileName2(QString file);
	void setMergeFileName3(QString file);
	void makeOutFileName();
	void mergeRtcmImuFile();
	void mergeRoverBaseFile();
	int readOneRtcm();
	int readOneImu();
	void readOneRover();
	void readOneRover2();
	void readOneBase();
	void UpdateProcess(int size);
	int getFileSize(FILE * file);
private:
	bool m_isStop;
	int m_MergeFormat;
	QTime m_TimeCounter;
	QString m_MergeFileName1;
	QString m_MergeFileName2;
	QString m_MergeFileName3;
	QString m_OutFileName;
	QString m_OutLogName;
	FILE * m_rtcmFile;
	FILE * m_imuFile;
	FILE * m_roverFile;
	FILE * m_rover2File;
	FILE * m_baseFile;
	FILE * m_outFile;
	FILE * m_logFile;
	int m_RawFilesSize;
	int m_RawFilesReadSize;

	QByteArray m_roverbuff;
	QByteArray m_rover2buff;
	QByteArray m_basebuff;
	QByteArray m_imubuff;
	uint8_t m_outbuff[MAX_BUFFER_SIZE];
	uint32_t m_outlen;
	gnss_rtcm_t m_gnss;
	gnss_rtcm_t m_gnss2;

	uint32_t last_TimeOfWeek;

	uint32_t per10Hz;
	uint32_t last_per10Hz;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};

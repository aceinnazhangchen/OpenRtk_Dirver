#pragma once

#include <QThread>
#include <QTime>
#include "decoder\mixed_raw.h"
#include "decoder\rtcm.h"
#include "decoder\imu_raw.h"

class MergeThread : public QThread
{
	Q_OBJECT

public:
	MergeThread(QObject *parent);
	~MergeThread();
	void run();
	void stop();
	void setRtcmFileName(QString file);
	void setImuFileName(QString file);
	void makeOutFileName();
	void mergeFile();
	int readOneRtcm();
	int readOneImu();
	void UpdateProcess(int size);
	int getFileSize(FILE * file);
private:
	bool m_isStop;
	QTime m_TimeCounter;
	QString m_RtcmFileName;
	QString m_ImuFileName;
	QString m_OutFileName;
	QString m_OutLogName;
	FILE * m_rtcmFile;
	FILE * m_imuFile;
	FILE * m_outFile;
	FILE * m_logFile;
	int m_RawFilesSize;
	int m_RawFilesReadSize;

	QByteArray m_roverbuff;
	QByteArray m_basebuff;
	QByteArray m_imubuff;
	uint8_t m_outbuff[MAX_BUFFER_SIZE];
	uint32_t m_outlen;
	gnss_rtcm_t m_gnss;

	uint32_t last_TimeOfWeek;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};

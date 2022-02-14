#pragma once

#include <QThread>
#include <QTime>
#include <QList>
#include "ins401.h"
#include "Ins401_Analysis.h"
#include "RTK330LA_Analysis.h"
#include "E2E_protocol.h"
#include "NPOS122_decoder.h"

//ºÍUIÖÐË³ÐòÒ»ÖÂ
enum emDecodeFormat {
	emDecodeFormat_OpenRTK330LI,
	emDecodeFormat_RTK330LA,
	emDecodeFormat_Mixed_Raw,
	emDecodeFormat_Imu,
	emDecodeFormat_Ins401,
	emDecodeFormat_E2E_Protocol,
	emDecodeFormat_RTCM_EPVT,
	emDecodeFormat_Convbin,
	emDecodeFormat_RTCM_2_HEX,
	emDecodeFormat_Beidou,
	emDecodeFormat_NPOS112,
};

class DecodeThread : public QThread
{
	Q_OBJECT

public:
	DecodeThread(QObject *parent);
	~DecodeThread();
	void run();
	void stop();
	void setFileFormat(int format);
	void setFileName(QString file);
	void setShowTime(bool show);
	void setKmlFrequency(int frequency);
	void setDateTimeStr(QString time);
	void setDateTime(QDateTime time);
	void setMIFileSwitch(bool write);
protected:
	void makeOutPath(QString filename);
	void decode_openrtk_user();
	void decode_openrtk_inceptio();
	void decode_mixed_raw();
	void decode_imu();
	void decode_ins401();
	void decode_e2e_protocol();
	void decode_rtcm_epvt();
	void decode_rtcm_convbin();
	void decode_rtcm_2_hex();
	void decode_beidou();
	void decode_npos112();
private:
	bool m_isStop;
	int m_FileFormat;
	QString m_FileName;
	QString m_OutBaseName;
	QTime m_TimeCounter;
	Ins401_Tool::Ins401_decoder* ins401_decoder;
	E2E::E2E_protocol* e2e_deocder;
	NPOS122_Tool::NPOS122_decoder* npos122_decoder;
	bool m_show_time;
	int ins_kml_frequency;
	QString m_datatime_str;
	QDateTime m_datatime;
public:
	bool m_static_point_ecp;
	Ins401_Tool::Ins401_Analysis* m_Ins401_Analysis;
	RTK330LA_Tool::RTK330LA_Analysis* m_RTK330LA_Analysis;
signals:
	void sgnProgress(int present, int msecs);
	void sgnFinished();
};

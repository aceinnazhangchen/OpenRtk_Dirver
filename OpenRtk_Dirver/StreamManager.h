#pragma once

#include <QObject>
#include <mutex>
#include <vector>
#include "SingleStream.h"
#include <QDir>
#include <QTimer>
#include <QTime>
#include "rtcm.h"
#include "openrtk_user.h"

#define MAX_STREAM_NUM 3
enum emPort {
	emPort_1 = 0,
	emPort_2 = 1,
	emPort_3 = 2,
};

enum emStreamType
{
	emStreamType_Serial = 1,
};

enum emRtkAction {
	emRtkLogFile = 0,
	emRtkReplayFile = 1,
};

enum emModelType {
	emModel_OpenRTK330LI = 1,
	emModel_RTK330LA = 2,
};

struct SerialConfig {
	QString port;
	int port_index;
	int baud;
	int bitnum;
	int bitnum_index;
	int parity;
	int stop;
	SerialConfig() {
		port.clear();
		port_index = 0;
		baud = 460800;
		bitnum = 8;
		bitnum_index = 0;
		parity = 0;
		stop = 0;
	}
};

struct StreamConfig {
	int stream_type;
	SerialConfig m_serial;
	StreamConfig() {
		stream_type = emStreamType_Serial;
	}
};

class StreamManager : public QObject
{
	Q_OBJECT

private:
	StreamManager(QObject *parent);
	~StreamManager();
public:
	static StreamManager* Instance();
	static void createInstance();
private:
	static StreamManager* m_instance;
	static std::once_flag m_flag;
public:
	StreamConfig& GetStreamConfig(int index);
	bool Open();
	void Close();
	void SetModelType(int model);
	void MakeLogPath();
	QDir GetLogPath();
	void LogOpenRTK();
	void StartReplayOpenRTK();
	void StopReplayOpenRTK();
	void ShowReadWriteSize(int index, int read, int write);
	void SetRtkAction(emRtkAction action);
	void SetReplayFileName(QString& filename);
	void InitTimer();
	bool SendReplayData();
	void SendPackage(int stn);
	void ReadReplayFileByTime();
	int ReadOnePackage();
	void UpdateProcess(int size);
	void OpenReplayFile();
	void SetDecode(bool isDecoding);
public slots:
	void onStream(int index, const QByteArray& data);
	void onTimerTimeout();
	void onStep();
private:
	int m_ModelType;
	QDir logPath;
	StreamConfig m_StreamConfigList[MAX_STREAM_NUM];
	SingleStream* m_StreamList[MAX_STREAM_NUM];
	QString cmd_response;
	emRtkAction m_RtkAction;
	QString m_ReplayFileName;
	QFile m_ReplayFile;
	int64_t m_ReplayFileSize;
	int64_t m_ReplayFileReadSize;
	QTimer *m_timer;	
	QTime m_TimeCounter;

	QByteArray m_roverbuff;
	QByteArray m_basebuff;
	QByteArray m_imubuff;
	FILE* m_logFile;
	bool m_isDecodeing;

	bool m_replayPort2Ready;
	bool m_replayPort3Ready;

	uint32_t send_imu_time;

	user_s1_t m_last_imu;
	gnss_rtcm_t gnss;
signals:
	void sgnShowReadWriteSize(int index, int read, int write);
	void sgnReplayProcess(int process,int mSecs);
	void sgnDecodeProcess(int process, int mSecs);
	void sgnFinishReplay();
};

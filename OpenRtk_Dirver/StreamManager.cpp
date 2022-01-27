#include "StreamManager.h"
#include <thread>
#include <mutex>
#include <QDebug>
#include <QDateTime>
#include <QJsonDocument>
#include "mixed_raw.h"

#define CMD_GET_CONFIG "get configuration\r\n"
#define CMD_LOG_ON "log debug on\r\n"
#define CMD_LOG_OFF "log debug off\r\n"
#define CMD_REPLAY_ON "replay on"
#define CMD_REPLAY_OFF "replay off"

StreamManager * StreamManager::m_instance = NULL;
std::once_flag      StreamManager::m_flag;

StreamManager::StreamManager(QObject *parent)
	: QObject(parent)
	, m_RtkAction(emRtkLogFile)
	, m_ModelType(emModel_OpenRTK330LI)
	, m_ReplayFileSize(0)
	, m_ReplayFileReadSize(0)
	, m_logFile(NULL)
	, m_isDecodeing(false)
	, m_replayPort2Ready(false)
	, m_replayPort3Ready(false)
	, send_imu_time(0)
{
	memset(&m_last_imu, 0, sizeof(m_last_imu));
	memset(&gnss, 0, sizeof(gnss));
	for (int i = 0; i < MAX_STREAM_NUM; i++) {
		m_StreamList[i] = new SingleStream(this);
		connect(m_StreamList[i], SIGNAL(sgnStream(int, const QByteArray&)), this, SLOT(onStream(int, const QByteArray&)));
	}
	InitTimer();
}

StreamManager::~StreamManager()
{
}

StreamManager* StreamManager::Instance()
{
	if (m_instance == NULL)
	{
		try
		{
			std::call_once(m_flag, createInstance);
		}
		catch (...)
		{
			qDebug("CreateInstance error\n");
		}
	}
	return m_instance;
}

void StreamManager::createInstance()
{
	m_instance = new(std::nothrow) StreamManager(NULL);
	if (NULL == m_instance)
	{
		throw std::exception();
	}
}

StreamConfig & StreamManager::GetStreamConfig(int index)
{
	return m_StreamConfigList[index];
}

bool StreamManager::Open()
{
	if (is_aceinna_decoding()) return false;
	set_aceinna_decoding(1);
	for (int i = 0; i < MAX_STREAM_NUM; i++) {
		if (m_StreamConfigList[i].stream_type == emStreamType_Serial) {
			if (m_StreamConfigList[i].m_serial.port.isEmpty()) {
				return false;
			}
		}
	}
	MakeLogPath();
	for (int i = 0; i < MAX_STREAM_NUM; i++) {
		if (m_StreamList[i]) {
			m_StreamList[i]->Open(i);
		}
	}
	if (m_RtkAction == emRtkLogFile) {
		LogOpenRTK();
	}
	else if (m_RtkAction == emRtkReplayFile) {
		StartReplayOpenRTK();
	}
	return true;
}

void StreamManager::Close()
{
	if (m_RtkAction == emRtkReplayFile) {
		StopReplayOpenRTK();
	}
	for (int i = 0; i < MAX_STREAM_NUM; i++) {
		if (m_StreamList[i]) {
			m_StreamList[i]->Close();
		}
	}
	set_aceinna_decoding(0);
}

void StreamManager::SetModelType(int model)
{
	m_ModelType = model;
}

void StreamManager::MakeLogPath()
{
	QDateTime currentTime = QDateTime::currentDateTime();
	QString strData = currentTime.toString("yyyy-MM-dd_hh-mm-ss");
	logPath = (QDir::currentPath() + QDir::separator() + "Data" + QDir::separator() + "openrtk_" + strData + QDir::separator());
	if (!logPath.exists()) {
		logPath.mkpath(logPath.absolutePath());
	}
}

QDir StreamManager::GetLogPath() {
	return logPath;
}

void StreamManager::LogOpenRTK()
{
	if (m_ModelType == emModel_OpenRTK330LI) {
		m_StreamList[emPort_3]->SendCmdAndWait(CMD_GET_CONFIG);
		cmd_response.clear();
	}
}

void StreamManager::StartReplayOpenRTK()
{
	if (m_ModelType == emModel_OpenRTK330LI) {
		m_StreamList[emPort_2]->SendCmdAndWait(CMD_REPLAY_ON);
		m_StreamList[emPort_3]->SendCmdAndWait(CMD_REPLAY_ON);
		m_replayPort2Ready = false;
		m_replayPort3Ready = false;
	}
	else if (m_ModelType == emModel_RTK330LA) {
		OpenReplayFile();
		m_timer->start();
	}
}

void StreamManager::StopReplayOpenRTK()
{
	if (m_timer->isActive()) m_timer->stop();
	if (m_ReplayFile.isOpen()) m_ReplayFile.close();
	if (m_ModelType == emModel_OpenRTK330LI) {
		m_StreamList[emPort_2]->SendCmd(CMD_REPLAY_OFF);
	}
	if (m_logFile) fclose(m_logFile); m_logFile = NULL;
}

void StreamManager::onStream(int index, const QByteArray& data) {
	SingleStream *obj = (SingleStream*)sender();
	if (obj->GetIndex() == emPort_3) {
		if (obj->IsWaitting()) {
			if (obj->GetWaittingCmd() == CMD_GET_CONFIG) {
				cmd_response.append(data);
				QJsonParseError jsonError;
				QJsonDocument doucement = QJsonDocument::fromJson(cmd_response.toUtf8(), &jsonError);
				if (!doucement.isNull() && (jsonError.error == QJsonParseError::NoError))
				{
					QFile configfile(logPath.absolutePath() + QDir::separator() + "configuration.json");
					configfile.open(QIODevice::WriteOnly);
					configfile.write(cmd_response.toUtf8());
					configfile.close();
					obj->ReleaseWaitting();
					obj->SendCmd(CMD_LOG_ON);
				}
			}
			else if (obj->GetWaittingCmd() == CMD_REPLAY_ON) {
				QString ret_cmd(data);
				if (ret_cmd == CMD_REPLAY_ON) {
					obj->ReleaseWaitting();
					m_replayPort3Ready = true;
					if (m_replayPort2Ready && m_replayPort3Ready) {
						OpenReplayFile();
						m_timer->start();
					}
				}
			}
		}
	}else if (obj->GetIndex() == emPort_2) {
		if (obj->IsWaitting()) {
			if (obj->GetWaittingCmd() == CMD_REPLAY_ON) {
				QString ret_cmd(data);
				if (ret_cmd == CMD_REPLAY_ON){
					obj->ReleaseWaitting();
					m_replayPort2Ready = true;
					if (m_replayPort2Ready && m_replayPort3Ready) {
						OpenReplayFile();
						m_timer->start();
					}
				}
			}
		}
	}
}

void StreamManager::ShowReadWriteSize(int index, int read, int write) {
	emit sgnShowReadWriteSize(index, read, write);
}

void StreamManager::SetRtkAction(emRtkAction action)
{
	m_RtkAction = action;
}

void StreamManager::SetReplayFileName(QString filename)
{
	m_ReplayFileName = filename;
}

void StreamManager::onTimerTimeout()
{
	if (SendReplayData() == false) {
		StopReplayOpenRTK();
		emit sgnFinishReplay();
	}
}

void StreamManager::onStep()
{
	SendReplayData();
}

void StreamManager::InitTimer()
{
	m_timer = new QTimer(this);
	//设置定时器是否为单次触发。默认为 false 多次触发
	m_timer->setSingleShot(false);
	m_timer->stop();
	m_timer->setInterval(10);
	connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimerTimeout()));
}

bool StreamManager::SendReplayData()
{
	bool ret = false;
	if (m_ReplayFile.isOpen()) {
		if (!m_ReplayFile.atEnd())
		{
			ret = true;
			if (send_imu_time) {
				send_imu_time += 10;
			}
			if (send_imu_time == 0 || send_imu_time >= m_last_imu.GPS_TimeOfWeek + 10) {
				ReadReplayFileByTime();
			}
			else if (send_imu_time >= m_last_imu.GPS_TimeOfWeek) {
				SendPackage(TYPE_IMU);
			}
		}
	}
	return ret;
}

void StreamManager::SendPackage(int stn) {
	if (TYPE_IMU == stn) {
		if (m_imubuff.size() > 0) {
			UpdateProcess(m_imubuff.size());
			if (m_ModelType == emModel_OpenRTK330LI) {
				m_StreamList[emPort_3]->Write(m_imubuff);
			}
			else if (m_ModelType == emModel_RTK330LA) {
				m_StreamList[emPort_1]->Write(m_imubuff);
			}
			m_imubuff.clear();
		}
	}
	else if (TYPE_ROV == stn) {
		if (m_roverbuff.size() > 0) {
			UpdateProcess(m_roverbuff.size());
			if (m_logFile) fprintf(m_logFile, "rover,time:%f,size:%d\n", gnss.rcv[stn - 1].time.time + gnss.rcv[stn - 1].time.sec, m_roverbuff.size());
			//send data;
			if (m_ModelType == emModel_OpenRTK330LI) {
				m_StreamList[emPort_2]->Write(m_roverbuff);
			}
			else if (m_ModelType == emModel_RTK330LA) {
				m_StreamList[emPort_1]->Write(m_roverbuff);
			}
			m_roverbuff.clear();
		}
	}
	else if (TYPE_BAS == stn) {
		if (m_basebuff.size() > 0) {
			UpdateProcess(m_basebuff.size());
			if (m_logFile) fprintf(m_logFile, "base,time:%f,size:%d\n", gnss.rcv[stn - 1].time.time + gnss.rcv[stn - 1].time.sec, m_basebuff.size());
			//send data;
			if (m_ModelType == emModel_OpenRTK330LI) {
				m_StreamList[emPort_2]->Write(m_basebuff);
			}
			else if (m_ModelType == emModel_RTK330LA) {
				m_StreamList[emPort_1]->Write(m_basebuff);
			}
			m_basebuff.clear();
		}
	}
}

void StreamManager::ReadReplayFileByTime() 
{
	//如果是rtcm数据就连续发送,直到发送imu数据就中断
	for (int i = 0; i < 3; i++) {
		int stn = ReadOnePackage();
		if (TYPE_ROV == stn) {
			SendPackage(stn);
		}
		else if (TYPE_BAS == stn) {
			SendPackage(stn);
		}
		else if (TYPE_IMU == stn) {
			if (send_imu_time >= m_last_imu.GPS_TimeOfWeek) {
				SendPackage(stn);
			}
			break;
		}
	}
}

int StreamManager::ReadOnePackage()
{
	uint8_t outbuff[1024] = { 0 };
	uint32_t outlen = 0;
	bool is_complete = false;
	int stn = 0;
	while (true) {
		if (m_ReplayFile.atEnd()) break;
		QByteArray bytes = m_ReplayFile.read(1);
		stn = input_aceinna_format_raw(bytes[0], outbuff, &outlen);
		if (TYPE_ROV == stn) {
			m_roverbuff.append((char*)outbuff, outlen);
			//if (m_logFile) fprintf(m_logFile, "$ROV,len:%d\n", outlen);
		}
		else if (TYPE_BAS == stn) {
			m_basebuff.append((char*)outbuff, outlen);
			//if (m_logFile) fprintf(m_logFile, "$BAS,len:%d\n", outlen);
		}
		else if (TYPE_IMU == stn) {
			//log
			memcpy(&m_last_imu, outbuff + ACEINNA_HEAD_SIZE, outlen - ACEINNA_HEAD_SIZE);
			if (m_logFile) fprintf(m_logFile, "$IMU,len:%d,%d,%d\n", outlen, m_last_imu.GPS_Week, m_last_imu.GPS_TimeOfWeek);
			//send data;
			if (send_imu_time == 0) {
				send_imu_time = m_last_imu.GPS_TimeOfWeek;
			}
			m_imubuff.append((char*)outbuff, outlen);
			break;
		}
		if (TYPE_ROV == stn || TYPE_BAS == stn) {
			for (int j = 0; j < outlen; j++) {
				int ret = input_rtcm3(outbuff[j], stn - 1, &gnss);
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

void StreamManager::UpdateProcess(int size) {
	m_ReplayFileReadSize += size;
	int nProcess = m_ReplayFileReadSize * 10000 / m_ReplayFileSize;
	if (m_isDecodeing) {
		emit sgnDecodeProcess(nProcess, m_TimeCounter.elapsed());
	}
	else {
		emit sgnReplayProcess(nProcess, m_TimeCounter.elapsed());
	}
}

void StreamManager::OpenReplayFile()
{
	if (m_ReplayFile.isOpen()) return;
	if (m_ReplayFileName.isEmpty()) return;
	m_ReplayFile.setFileName(m_ReplayFileName);
	m_ReplayFile.open(QIODevice::ReadOnly);
	m_ReplayFileSize = m_ReplayFile.size();
	m_ReplayFileReadSize = 0;
	send_imu_time = 0;
	emit sgnReplayProcess(0, 0);
	m_TimeCounter.start();
	set_output_aceinna_file(0);
	memset(&gnss, 0, sizeof(gnss_rtcm_t));
	QString logFileName = m_ReplayFileName + ".out";
	m_logFile = fopen(logFileName.toLocal8Bit().data(), "w");
}

void StreamManager::SetDecode(bool isDecoding)
{
	m_isDecodeing = isDecoding;
}
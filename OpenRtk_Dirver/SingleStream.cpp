#include "SingleStream.h"
#include "StreamManager.h"
#include <QDateTime>
#include <QDir>

SingleStream::SingleStream(QObject *parent)
	: QObject(parent)
	, m_isUserOpen(false)
	, m_Index(0)
	, m_islogFile(true)
	, m_isWaitting(false)
	, m_nReadDateSize(0)
	, m_nWriteDateSize(0)
{
	m_QSerialPort = new QSerialPort(this);
	connect(m_QSerialPort, SIGNAL(readyRead()), this, SLOT(onReadReady()));
}

SingleStream::~SingleStream()
{
}

void SingleStream::Open(int nIndex)
{
	if (m_isUserOpen)return;
	m_isUserOpen = true;
	m_nReadDateSize = 0;
	m_nWriteDateSize = 0;
	m_Index = nIndex;	
	StreamConfig& config = StreamManager::Instance()->GetStreamConfig(m_Index);
	if (config.stream_type == emStreamType_Serial) {
		m_QSerialPort->setPortName(config.m_serial.port);
		m_QSerialPort->setBaudRate(config.m_serial.baud);
		switch (config.m_serial.bitnum)
		{
		case 5: m_QSerialPort->setDataBits(QSerialPort::Data5); break;
		case 6: m_QSerialPort->setDataBits(QSerialPort::Data6); break;
		case 7: m_QSerialPort->setDataBits(QSerialPort::Data7); break;
		case 8: m_QSerialPort->setDataBits(QSerialPort::Data8); break;
		default: break;
		}
		switch (config.m_serial.parity)
		{
		case 0: m_QSerialPort->setParity(QSerialPort::NoParity); break;
		case 1: m_QSerialPort->setParity(QSerialPort::OddParity); break;
		case 2: m_QSerialPort->setParity(QSerialPort::EvenParity); break;
		case 3: m_QSerialPort->setParity(QSerialPort::SpaceParity); break;
		case 4: m_QSerialPort->setParity(QSerialPort::MarkParity); break;
		default: break;
		}
		switch (config.m_serial.stop)
		{
		case 0: m_QSerialPort->setStopBits(QSerialPort::OneStop); break;
		case 1: m_QSerialPort->setStopBits(QSerialPort::OneAndHalfStop); break;
		case 2: m_QSerialPort->setStopBits(QSerialPort::TwoStop); break;
		default: break;
		}
		m_QSerialPort->setFlowControl(QSerialPort::NoFlowControl);
		m_QSerialPort->open(QIODevice::ReadWrite);
		OpenStreamLogFile();
	}	
}
void SingleStream::Close()
{
	StreamConfig& config = StreamManager::Instance()->GetStreamConfig(m_Index);
	if (config.stream_type == emStreamType_Serial)
	{
		m_QSerialPort->clear();
		m_QSerialPort->close();
	}
	CloseStreamLogFile();
	m_isUserOpen = false;
}

void SingleStream::Write(const QByteArray & data)
{
	if (!m_isUserOpen)return;
	StreamConfig& config = StreamManager::Instance()->GetStreamConfig(m_Index);
	if (config.stream_type == emStreamType_Serial)
	{
		m_QSerialPort->write(data);
		m_QSerialPort->waitForBytesWritten();
		m_nWriteDateSize += data.size();
		StreamManager::Instance()->ShowReadWriteSize(m_Index, m_nReadDateSize, m_nWriteDateSize);
	}
}

void SingleStream::LogFile(bool logFile)
{
	m_islogFile = logFile;
}

int SingleStream::GetIndex()
{
	return m_Index;
}

void SingleStream::SendCmd(QString cmd)
{
	Write(cmd.toLocal8Bit());
}

void SingleStream::SendCmdAndWait(QString cmd)
{
	m_isWaitting = true;
	m_CurrentCmd = cmd;
	Write(cmd.toLocal8Bit());
}

void SingleStream::ReleaseWaitting()
{
	m_isWaitting = false;
}

bool SingleStream::IsWaitting()
{
	return m_isWaitting;
}

QString & SingleStream::GetWaittingCmd()
{
	return m_CurrentCmd;
}

QString SingleStream::GetName()
{
	StreamConfig& config = StreamManager::Instance()->GetStreamConfig(m_Index);
	if (config.stream_type == emStreamType_Serial) {
		return QString::asprintf("Serial_%d_%s_", m_Index + 1, qPrintable(m_QSerialPort->portName()));
	}
	else {
		return QString::asprintf("File_%d_", m_Index + 1);
	}
}

void SingleStream::OpenStreamLogFile() {
	QDir targetDir = StreamManager::Instance()->GetLogPath();
	QDateTime currentTime = QDateTime::currentDateTime();
	QString strTime = currentTime.toString("yyyy-MM-dd_hh-mm-ss");
	m_StreamLogFile.setFileName(targetDir.absolutePath() + QDir::separator() + GetName() + strTime  + ".bin");
	m_StreamLogFile.open(QIODevice::WriteOnly);
}

void SingleStream::CloseStreamLogFile() {
	if (m_StreamLogFile.isOpen()) m_StreamLogFile.close();
}

void SingleStream::WriteStreamFileLog(const QByteArray & data) {
	if (m_StreamLogFile.isOpen())
	{
		m_StreamLogFile.write(data);
	}
}

void SingleStream::onReadReady()
{
	QByteArray byteArray = m_QSerialPort->readAll();
	m_nReadDateSize += byteArray.size();
	StreamManager::Instance()->ShowReadWriteSize(m_Index, m_nReadDateSize, m_nWriteDateSize);
	if (m_isWaitting) {
		emit sgnStream(m_Index, byteArray);
	}
	else {
		WriteStreamFileLog(byteArray);
	}
}

#pragma once

#include <QObject>
#include <QtSerialPort/QSerialPort>  
#include <QtSerialPort/QSerialPortInfo>
#include <QFile>


class SingleStream : public QObject
{
	Q_OBJECT

public:
	SingleStream(QObject *parent);
	~SingleStream();
	void Open(int nIndex);
	void Close();
	void Write(const QByteArray & data);
	void LogFile(bool logFile);
	int GetIndex();
	void SendCmd(QString cmd);
	void SendCmdAndWait(QString cmd);
	void ReleaseWaitting();
	bool IsWaitting();
	QString& GetWaittingCmd();
private:
	QString GetName();
	void OpenStreamLogFile();
	void CloseStreamLogFile();
	void WriteStreamFileLog(const QByteArray & data);
private:
	bool m_isUserOpen;
	int m_Index;
	int m_nReadDateSize;
	int m_nWriteDateSize;
	QFile m_StreamLogFile;
	QSerialPort* m_QSerialPort;

	bool m_islogFile;
	bool m_isWaitting;
	QString m_CurrentCmd;
public slots:
	void onReadReady();
signals:
	void sgnStream(int, const QByteArray&);
};

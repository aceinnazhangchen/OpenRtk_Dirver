#pragma once

#include <thread>
#include <mutex>
#include <iostream>
#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

class ConfigFile : public QObject
{
	Q_OBJECT

private:
	ConfigFile(QObject *parent);
	~ConfigFile();
public:
	static ConfigFile* getInstance();
	static void createInstance();

private:
	static ConfigFile* m_instance;
	static std::once_flag m_flag;
public:
	void readConfigFile();
	void writeConfigFile();
	QJsonObject& getConfig();
private:
	QJsonObject m_ConfigJson;
};

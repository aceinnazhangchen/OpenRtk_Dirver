#pragma once

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

class JsonFileLoader : public QObject
{
	Q_OBJECT

public:
	JsonFileLoader(QObject *parent);
	~JsonFileLoader();
public:
	bool readJsonArrayFile(QString filename);
	QJsonObject& getConfig();
private:
	QJsonObject m_ConfigJson;

};

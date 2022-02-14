#include "ConfigFile.h"
#include <QFile>
#include <QDebug>

ConfigFile * ConfigFile::m_instance = NULL;
std::once_flag      ConfigFile::m_flag;

ConfigFile::ConfigFile(QObject *parent)
	: QObject(parent)
{
	readConfigFile();
}

ConfigFile::~ConfigFile()
{
}

ConfigFile* ConfigFile::getInstance()
{
	if (m_instance == NULL)
	{
		try
		{
			std::call_once(m_flag, createInstance);
		}
		catch (...)
		{
			qDebug() << "CreateInstance error\n";
		}
	}
	return m_instance;
}

void ConfigFile::createInstance()
{
	m_instance = new(std::nothrow) ConfigFile(NULL);
	if (NULL == m_instance)
	{
		throw std::exception();
	}
}

void ConfigFile::readConfigFile()
{
	QFile file("./content_aceinna_config.json");
	file.open(QIODevice::ReadOnly | QIODevice::Text);
	if (!file.isOpen()) return;
	QString value = file.readAll();
	file.close();

	QJsonParseError jsonError;
	QJsonDocument doucement = QJsonDocument::fromJson(value.toUtf8(), &jsonError);

	if (!doucement.isNull() && (jsonError.error == QJsonParseError::NoError))
	{
		if (doucement.isObject())
		{
			m_ConfigJson = doucement.object();
		}
	}
}

void ConfigFile::writeConfigFile()
{
	QJsonDocument document;
	document.setObject(m_ConfigJson);
	QByteArray byteArray = document.toJson(QJsonDocument::Indented);
	QString jsonStr = (byteArray);

	QFile file("./content_aceinna_config.json");
	if (!file.open(QIODevice::ReadWrite | QIODevice::Text | QIODevice::Truncate))
	{
		qDebug() << "file error";
	}
	QTextStream in(&file);
	in << jsonStr;
	file.close();
}

QJsonObject & ConfigFile::getConfig()
{
	return m_ConfigJson;
}

#include "JsonFileLoader.h"
#include <QFile>
#include <QMessageBox>

JsonFileLoader::JsonFileLoader(QObject *parent)
	: QObject(parent)
{
}

JsonFileLoader::~JsonFileLoader()
{
}

bool JsonFileLoader::readJsonArrayFile(QString filename)
{
	bool ret = false;
	QFile file(filename);
	file.open(QIODevice::ReadOnly | QIODevice::Text);
	if (!file.isOpen()) return ret;
	QString value = file.readAll();
	file.close();

	QJsonParseError jsonError;
	QJsonDocument doucement = QJsonDocument::fromJson(value.toUtf8(), &jsonError);

	if (!doucement.isNull() && (jsonError.error == QJsonParseError::NoError))
	{
		if (doucement.isArray() && doucement.array().size() > 0)
		{
			if (doucement.array()[0].isObject()) {
				m_ConfigJson = doucement.array()[0].toObject();
				ret = true;
			}
		}
	}
	if (!ret) {
		QMessageBox::critical(NULL, "Error", "Can't Open Json File!\r\n Please check the file!");
	}
	return ret;
}

QJsonObject & JsonFileLoader::getConfig()
{
	// TODO: 在此处插入 return 语句
	return m_ConfigJson;
}

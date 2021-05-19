#include "StreamConfigUI.h"
#include <QtSerialPort/QSerialPortInfo>
#include "StreamManager.h"

StreamConfigUI::StreamConfigUI(QWidget *parent)
	: QWidget(parent)
	, m_CurrentIndex(0)
{
	ui.setupUi(this);
	ui.tabWidget->tabBar()->hide();
	connect(ui.m_buttonOK, SIGNAL(clicked()), this, SLOT(onCloseUI()));
}

StreamConfigUI::~StreamConfigUI()
{
}

void StreamConfigUI::onCloseUI()
{
	SaveConfig();
	close();
	emit sgnClosed();
}

void StreamConfigUI::SaveConfig() {
	StreamConfig& config = StreamManager::Instance()->GetStreamConfig(m_CurrentIndex);
	if (ui.m_comboBoxType->currentIndex() == 0) {
		config.stream_type = emStreamType_Serial;
		config.m_serial.port = ui.PortBox->currentText();
		config.m_serial.port_index = ui.PortBox->currentIndex();
		config.m_serial.baud = ui.BaudBox->currentText().toInt();
		config.m_serial.bitnum = ui.BitNumBox->currentText().toInt();
		config.m_serial.bitnum_index = ui.BitNumBox->currentIndex();
		config.m_serial.parity = ui.ParityBox->currentIndex();
		config.m_serial.stop = ui.StopBox->currentIndex();
	}
}

void StreamConfigUI::LoadConfig() {
	StreamConfig& config = StreamManager::Instance()->GetStreamConfig(m_CurrentIndex);
	if (config.stream_type == emStreamType_Serial) {
		ui.PortBox->setCurrentIndex(config.m_serial.port_index);
		ui.BaudBox->setCurrentText(QString::number(config.m_serial.baud));
		ui.BitNumBox->setCurrentIndex(config.m_serial.bitnum_index);
		ui.ParityBox->setCurrentIndex(config.m_serial.parity);
		ui.StopBox->setCurrentIndex(config.m_serial.stop);
	}
}

void StreamConfigUI::SearchComPort()
{
	int current_index = ui.PortBox->currentIndex();
	ui.PortBox->clear();
	foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())//通过QSerialPortInfo查找可用串口
	{
		ui.PortBox->addItem(info.portName());
	}
	current_index = current_index == -1 ? 0 : current_index;
	ui.PortBox->setCurrentIndex(current_index);
}

void StreamConfigUI::OpenIndex(int index)
{
	m_CurrentIndex = index;
	SearchComPort();
	LoadConfig();
	show();
}

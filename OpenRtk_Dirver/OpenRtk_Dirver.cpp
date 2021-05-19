#include "OpenRtk_Dirver.h"
#include "StreamManager.h"
#include <QtSerialPort/QSerialPortInfo>
#include <QDir>
#include <QUrl>
#include <QDesktopServices>
#include <QFileDialog>

#define KB 1024

QString FormatBytes(int byte) {
	QString number_str;
	double number = 0;;
	if (byte < KB) {
		number_str = QString::number(byte) + "B";
	}
	else if (byte >= KB && byte <= KB * KB) {
		number = double(byte) / KB;
		number_str = QString::number(number, 'f', 2) + "K";
	}
	else if (byte >= KB * KB && byte <= KB * KB * KB) {
		number = double(byte) / (KB * KB);
		number_str = QString::number(number, 'f', 2) + "M";
	}
	return number_str;
}

OpenRtk_Dirver::OpenRtk_Dirver(QWidget *parent)
    : QMainWindow(parent)
	, m_nStates(emStop)
{
    ui.setupUi(this);
	setAcceptDrops(true);
	ui.BaudBox->setCurrentIndex(3);
	onSearchComPort();
	ui.tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	for (int i = 0; i < MAX_STREAM_NUM; i++) {
		ui.tableWidget->insertRow(ui.tableWidget->rowCount());
		int rowIdx = ui.tableWidget->rowCount() - 1;
		QTableWidgetItem *item0 = new QTableWidgetItem(QString::asprintf("Port%d", i + 1));
		QTableWidgetItem *item1 = new QTableWidgetItem("0");
		QTableWidgetItem *item2 = new QTableWidgetItem("0");
		ui.tableWidget->setItem(rowIdx, 0, item0);
		ui.tableWidget->setItem(rowIdx, 1, item1);
		ui.tableWidget->setItem(rowIdx, 2, item2);
	}
	connect(ui.refresh_btn, SIGNAL(clicked()), this, SLOT(onSearchComPort()));
	connect(ui.open_btn, SIGNAL(clicked()), this, SLOT(onOpenClose()));
	connect(ui.clear_btn, SIGNAL(clicked()), this, SLOT(onClear()));
	connect(StreamManager::Instance(), SIGNAL(sgnShowReadWriteSize(int,int,int)),this,SLOT(onShowReadWriteSize(int, int, int)));
	connect(StreamManager::Instance(), SIGNAL(sgnReplayProcess(int, int)), this, SLOT(onReplayProcess(int, int)));
	connect(StreamManager::Instance(), SIGNAL(sgnFinishReplay()), this, SLOT(onOpenClose()));
	connect(ui.log_path_btn, SIGNAL(clicked()), this, SLOT(onOpenLogPathClicked()));
	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));

	ui.decode_btn->hide();

	m_DecodeToolsWidget = new DecodeTools(NULL);
	m_MergeTools = new MergeTools(NULL);
	m_AnalysisTools = new AnalysisTools(NULL);
	connect(ui.actionDecoder, &QAction::triggered, this, &OpenRtk_Dirver::onDecodeWidgetOpen);
	connect(ui.actionMerger, &QAction::triggered, this, &OpenRtk_Dirver::onMergeWidgetOpen);
	connect(ui.actionAnalysis, &QAction::triggered, this, &OpenRtk_Dirver::onAnalysisWidgetOpen);
}

OpenRtk_Dirver::~OpenRtk_Dirver()
{
}

void OpenRtk_Dirver::dragEnterEvent(QDragEnterEvent *event)
{
	event->acceptProposedAction();
}

void OpenRtk_Dirver::dropEvent(QDropEvent *event)
{
	QString name = event->mimeData()->urls().first().toLocalFile();
	ui.filepath_edt->setText(name);
}

void OpenRtk_Dirver::getConfigFromUI()
{
	int baud = ui.BaudBox->currentText().toInt();
	StreamConfig & config_1 = StreamManager::Instance()->GetStreamConfig(0);
	config_1.m_serial.baud = baud;
	config_1.m_serial.port = ui.com1_box->currentText();
	config_1.m_serial.port_index = ui.com1_box->currentIndex();
	StreamConfig & config_2 = StreamManager::Instance()->GetStreamConfig(1);
	config_2.m_serial.baud = baud;
	config_2.m_serial.port = ui.com2_box->currentText();
	config_2.m_serial.port_index = ui.com2_box->currentIndex();
	StreamConfig & config_3 = StreamManager::Instance()->GetStreamConfig(2);
	config_3.m_serial.baud = baud;
	config_3.m_serial.port = ui.com3_box->currentText();
	config_3.m_serial.port_index = ui.com3_box->currentIndex();
	if (ui.replay_chk->isChecked()) {
		StreamManager::Instance()->SetRtkAction(emRtkReplayFile);
		StreamManager::Instance()->SetReplayFileName(ui.filepath_edt->text());
	}
	else {
		StreamManager::Instance()->SetRtkAction(emRtkLogFile);
	}
}

void OpenRtk_Dirver::changePortName()
{
	for (int i = 0; i < MAX_STREAM_NUM; i++) {
		StreamConfig & config = StreamManager::Instance()->GetStreamConfig(i);
		QTableWidgetItem *item0 = ui.tableWidget->item(i, 0);
		item0->setText(config.m_serial.port);
	}
}

void OpenRtk_Dirver::onOpenClose()
{
	if (m_nStates == emStop)
	{
		getConfigFromUI();
		changePortName();

		if (StreamManager::Instance()->Open()) {
			m_nStates = emStart;
		}
	}
	else {
		StreamManager::Instance()->Close();
		m_nStates = emStop;
	}

	if (m_nStates == emStart)
	{
		ui.BaudBox->setDisabled(true);
		ui.com1_box->setDisabled(true);
		ui.com2_box->setDisabled(true);
		ui.com3_box->setDisabled(true);
		ui.replay_chk->setDisabled(true);
		ui.filepath_edt->setDisabled(true);
		ui.select_btn->setDisabled(true);
		ui.refresh_btn->setDisabled(true);
		ui.open_btn->setText("Close");
	}
	else
	{
		ui.BaudBox->setDisabled(false);
		ui.com1_box->setDisabled(false);
		ui.com2_box->setDisabled(false);
		ui.com3_box->setDisabled(false);
		ui.replay_chk->setDisabled(false);
		ui.filepath_edt->setDisabled(false);
		ui.select_btn->setDisabled(false);
		ui.refresh_btn->setDisabled(false);
		ui.open_btn->setText("Open");
	}
}

void OpenRtk_Dirver::onClear()
{
	for (int i = 0; i < MAX_STREAM_NUM; i++) {
		QTableWidgetItem *item1 = ui.tableWidget->item(i, 1);
		QTableWidgetItem *item2 = ui.tableWidget->item(i, 2);
		item1->setText("0");
		item2->setText("0");
	}
}

void OpenRtk_Dirver::onShowReadWriteSize(int index, int read, int write)
{
	QTableWidgetItem *item1 = ui.tableWidget->item(index, 1);
	QString number_str = FormatBytes(read);
	item1->setText(number_str);
	QTableWidgetItem *item2 = ui.tableWidget->item(index, 2);
	number_str = FormatBytes(write);
	item2->setText(number_str);
}

void OpenRtk_Dirver::onSearchComPort()
{
	ui.com1_box->clear(); 
	ui.com2_box->clear();
	ui.com3_box->clear();
	foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())//通过QSerialPortInfo查找可用串口
	{
		ui.com1_box->addItem(info.portName());
		ui.com2_box->addItem(info.portName());
		ui.com3_box->addItem(info.portName());
	}
	ui.com1_box->setCurrentIndex(0);
	ui.com2_box->setCurrentIndex(1);
	ui.com3_box->setCurrentIndex(2);
	getConfigFromUI();
}

void OpenRtk_Dirver::onOpenLogPathClicked()
{
	QDir path = StreamManager::Instance()->GetLogPath();
	if (path.isEmpty()) {
		QDesktopServices::openUrl(QUrl(QDir::currentPath(), QUrl::TolerantMode));
	}
	else {
		QDesktopServices::openUrl(QUrl(path.absolutePath(), QUrl::TolerantMode));
	}
}

void OpenRtk_Dirver::onSelectFileClicked() {
	QString current_path = ".";
	QString file_name = ui.filepath_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("RAW Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.filepath_edt->setText(path);
}

void OpenRtk_Dirver::onReplayProcess(int nProcess, int mSecs)
{
	ui.progressBar->setValue(nProcess);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.showtime_lb->setText(m_TimeShow.addMSecs(mSecs).toString("hh:mm:ss:zzz"));
}

void OpenRtk_Dirver::onDecodeWidgetOpen()
{
	m_DecodeToolsWidget->show();
}

void OpenRtk_Dirver::onMergeWidgetOpen()
{
	m_MergeTools->show();
}

void OpenRtk_Dirver::onAnalysisWidgetOpen()
{
	m_AnalysisTools->show();
}
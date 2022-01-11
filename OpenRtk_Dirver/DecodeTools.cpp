#include "DecodeTools.h"
#include <QDir>
#include <QUrl>
#include <QDesktopServices>
#include <QFileDialog>

DecodeTools::DecodeTools(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	setAcceptDrops(true);
	m_DecodeThread = new DecodeThread(this);
	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.decode_btn, SIGNAL(clicked()), this, SLOT(onDecodeClicked()));
	connect(m_DecodeThread, SIGNAL(sgnProgress(int, int)), this, SLOT(onProcess(int, int)));
	connect(m_DecodeThread, SIGNAL(sgnFinished()), this, SLOT(onFinished()));
	connect(ui.toolButton_setting, SIGNAL(clicked()), this, SLOT(onClickedSettingButton()));
	
	m_AnalysisConfigUI = new AnalysisConfigUI();
	m_AnalysisConfigUI->hide();

	ui.dateTimeEdit->setDate(QDate::currentDate());
}

DecodeTools::~DecodeTools()
{
	if (m_DecodeThread && m_DecodeThread->isRunning()) {
		m_DecodeThread->stop();
		m_DecodeThread->wait();
	}
}

void DecodeTools::setOperable(bool enable)
{
	ui.filepath_edt->setEnabled(enable);
	ui.select_btn->setEnabled(enable);
	//ui.decode_btn->setEnabled(enable);
	if (enable) {
		ui.decode_btn->setText("decode");
	}
	else {
		ui.decode_btn->setText("stop");
	}
}

void DecodeTools::dragEnterEvent(QDragEnterEvent * event)
{
	event->acceptProposedAction();
}

void DecodeTools::dropEvent(QDropEvent * event)
{
	if (ui.filepath_edt->isEnabled()) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.filepath_edt->setText(name);
	}
}

void DecodeTools::onSelectFileClicked() {
	QString current_path = ".";
	QString file_name = ui.filepath_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.filepath_edt->setText(path);
}

void DecodeTools::onDecodeClicked()
{
	if (m_DecodeThread->isRunning())
	{
		m_DecodeThread->stop();
		return;
	}
	QString filename = ui.filepath_edt->text();
	if (filename.isEmpty()) {
		return;
	}
	ui.progressBar->setValue(0);
	m_DecodeThread->setFileFormat(ui.fileformat_cmb->currentIndex());
	m_DecodeThread->setFileName(filename);
	m_DecodeThread->setShowTime(ui.time_checkBox->isChecked());
	switch (ui.frequency_cmb->currentIndex()) {
	case 0:
		m_DecodeThread->setKmlFrequency(1000);
		break;
	case 1:
		m_DecodeThread->setKmlFrequency(100);
		break;
	case 2:
		m_DecodeThread->setKmlFrequency(10);
		break;
	default:
		break;
	}
	if (ui.fileformat_cmb->currentIndex() == emDecodeFormat_Ins401) {
		m_AnalysisConfigUI->set_thres_Ins401(m_DecodeThread->m_Ins401_Analysis);
	}
	else if(ui.fileformat_cmb->currentIndex() == emDecodeFormat_RTK330LA) {
		m_AnalysisConfigUI->set_thres_RTK330LA(m_DecodeThread->m_RTK330LA_Analysis);
	}	
	m_DecodeThread->m_static_point_ecp = m_AnalysisConfigUI->isStaticTotalChecked();
	m_DecodeThread->setMIFileSwitch(m_AnalysisConfigUI->isMITableChecked());
	QString time = ui.dateTimeEdit->dateTime().toString("yyyy/MM/dd HH:mm:ss");
	m_DecodeThread->setDateTime(time);
	m_DecodeThread->start();
	setOperable(false);
}

void DecodeTools::onProcess(int present, int msecs)
{
	ui.progressBar->setValue(present);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.time_lb->setText(m_TimeShow.addMSecs(msecs).toString("hh:mm:ss:zzz"));
}

void DecodeTools::onFinished()
{
	setOperable(true);
}

void DecodeTools::onClickedSettingButton()
{
	m_AnalysisConfigUI->show();
}

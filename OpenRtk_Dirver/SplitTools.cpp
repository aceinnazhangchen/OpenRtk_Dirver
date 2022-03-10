#include "SplitTools.h"
#include "common.h"
#include <QDir>
#include <QUrl>
#include <QDesktopServices>
#include <QFileDialog>

SplitTools::SplitTools(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	setAcceptDrops(true);
	m_SplitThread = new SplitThread(this);
	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.split_btn, SIGNAL(clicked()), this, SLOT(onSplitClicked()));
	connect(m_SplitThread, SIGNAL(sgnProgress(int, int)), this, SLOT(onProcess(int, int)));
	connect(m_SplitThread, SIGNAL(sgnFinished()), this, SLOT(onFinished()));

	QDateTime current = QDateTime::currentDateTime();
	ui.dateTimeEdit_start->setDateTime(current.addSecs(-HOUR));
	ui.dateTimeEdit_end->setDateTime(current);
	ui.timeEdit->setTime(QTime(1, 0));
}

SplitTools::~SplitTools()
{
	if (m_SplitThread && m_SplitThread->isRunning()) {
		m_SplitThread->stop();
		m_SplitThread->wait();
	}
}

void SplitTools::setOperable(bool enable)
{
	ui.filepath_edt->setEnabled(enable);
	ui.select_btn->setEnabled(enable);
	ui.split_btn->setEnabled(enable);
}

void SplitTools::dragEnterEvent(QDragEnterEvent * event)
{
	event->acceptProposedAction();
}

void SplitTools::dropEvent(QDropEvent * event)
{
	if (ui.filepath_edt->isEnabled()) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.filepath_edt->setText(name);
	}
}

void SplitTools::onSelectFileClicked()
{
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

void SplitTools::onSplitClicked()
{
	if (m_SplitThread->isRunning())
	{
		return;
	}
	QString filename = ui.filepath_edt->text();
	if (filename.isEmpty()) {
		return;
	}
	ui.progressBar->setValue(0);
	m_SplitThread->setFileFormat(ui.fileformat_cmb->currentIndex());
	m_SplitThread->setFileName(filename);
	m_SplitThread->set_time_range(ui.dateTimeEdit_start->dateTime().toTime_t(), ui.dateTimeEdit_end->dateTime().toTime_t());
	QTime time = ui.timeEdit->time();
	int day = ui.spinBox_day->value();
	uint32_t time_silce = day * DAY + time.hour()*HOUR + time.minute()*MINUTE + time.second();
	m_SplitThread->set_time_silce(time_silce);
	m_SplitThread->start();
	setOperable(false);
}

void SplitTools::onProcess(int present, int msecs)
{
	ui.progressBar->setValue(present);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.time_lb->setText(m_TimeShow.addMSecs(msecs).toString("hh:mm:ss:zzz"));
}

void SplitTools::onFinished()
{
	setOperable(true);
}

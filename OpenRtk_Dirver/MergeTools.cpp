#include "MergeTools.h"
#include <QDir>
#include <QUrl>
#include <QDesktopServices>
#include <QFileDialog>

MergeTools::MergeTools(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	setAcceptDrops(true);
	m_MergeThread = new MergeThread(this);

	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.select2_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked2()));
	connect(ui.merge_btn, SIGNAL(clicked()), this, SLOT(onMergeClicked()));

	connect(m_MergeThread, SIGNAL(sgnProgress(int, int)), this, SLOT(onProcess(int, int)));
	connect(m_MergeThread, SIGNAL(sgnFinished()), this, SLOT(onFinished()));
}

MergeTools::~MergeTools()
{
	if (m_MergeThread && m_MergeThread->isRunning()) {
		m_MergeThread->stop();
		m_MergeThread->wait();
	}
}

void MergeTools::setOperable(bool enable)
{
	ui.filepath_edt->setEnabled(enable);
	ui.select_btn->setEnabled(enable);
	ui.filepath2_edt->setEnabled(enable);
	ui.select2_btn->setEnabled(enable);
	ui.merge_btn->setEnabled(enable);
}

void MergeTools::dragEnterEvent(QDragEnterEvent * event)
{
	event->acceptProposedAction();
}

void MergeTools::dropEvent(QDropEvent * event)
{
	if (ui.filepath_edt->isEnabled() && ui.filepath_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.filepath_edt->setText(name);
	}
	else if (ui.filepath2_edt->isEnabled() && ui.filepath2_edt->geometry().contains(event->pos())) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.filepath2_edt->setText(name);
	}
}

void MergeTools::onSelectFileClicked() {
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
void MergeTools::onSelectFileClicked2() {
	QString current_path = ".";
	QString file_name = ui.filepath2_edt->text();
	if (!file_name.isEmpty()) {
		current_path = QDir(file_name).absolutePath();
	}
	QString path = QFileDialog::getOpenFileName(this, tr("Open Files"), current_path, tr("Data Files(*.* )"));
	if (path.length() == 0) {
		return;
	}
	ui.filepath2_edt->setText(path);
}

void MergeTools::onMergeClicked() {
	if (m_MergeThread->isRunning())
	{
		return;
	}
	QString filename = ui.filepath_edt->text();
	QString filename2 = ui.filepath2_edt->text();
	if (filename.isEmpty() || filename2.isEmpty()) {
		return;
	}
	ui.progressBar->setValue(0);
	m_MergeThread->setRtcmFileName(filename);
	m_MergeThread->setImuFileName(filename2);
	m_MergeThread->start();
	setOperable(false);
}

void MergeTools::onProcess(int present, int msecs)
{
	ui.progressBar->setValue(present);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.time_lb->setText(m_TimeShow.addMSecs(msecs).toString("hh:mm:ss:zzz"));
}

void MergeTools::onFinished()
{
	setOperable(true);
}
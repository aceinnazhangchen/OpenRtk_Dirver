#include "AnalysisTools.h"
#include <QDir>
#include <QUrl>
#include <QDesktopServices>
#include <QFileDialog>
#include "StreamManager.h"

AnalysisTools::AnalysisTools(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	setAcceptDrops(true);
	m_AnalysisThread = new AnalysisThread(this);

	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.analysis_btn, SIGNAL(clicked()), this, SLOT(onAnalysisClicked()));

	connect(StreamManager::Instance(), SIGNAL(sgnDecodeProcess(int, int)), this, SLOT(onProcess(int, int)));
	connect(m_AnalysisThread, SIGNAL(sgnFinished()), this, SLOT(onFinished()));
}

AnalysisTools::~AnalysisTools()
{
	if (m_AnalysisThread && m_AnalysisThread->isRunning()) {
		m_AnalysisThread->stop();
		m_AnalysisThread->wait();
	}
}

void AnalysisTools::setOperable(bool enable)
{
	ui.filepath_edt->setEnabled(enable);
	ui.select_btn->setEnabled(enable);
	ui.analysis_btn->setEnabled(enable);
}

void AnalysisTools::dragEnterEvent(QDragEnterEvent * event)
{
	event->acceptProposedAction();
}

void AnalysisTools::dropEvent(QDropEvent * event)
{
	if (ui.filepath_edt->isEnabled()) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.filepath_edt->setText(name);
	}
}
void AnalysisTools::onSelectFileClicked() {
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

void AnalysisTools::onAnalysisClicked()
{
	if (m_AnalysisThread->isRunning())
	{
		return;
	}
	QString filename = ui.filepath_edt->text();
	if (filename.isEmpty()) {
		return;
	}
	ui.progressBar->setValue(0);
	m_AnalysisThread->setFileFormat(ui.fileformat_cmb->currentIndex());
	m_AnalysisThread->setFileName(filename);
	m_AnalysisThread->start();
	setOperable(false);
}

void AnalysisTools::onProcess(int present, int msecs)
{
	ui.progressBar->setValue(present);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.time_lb->setText(m_TimeShow.addMSecs(msecs).toString("hh:mm:ss:zzz"));
}

void AnalysisTools::onFinished()
{
	setOperable(true);
}
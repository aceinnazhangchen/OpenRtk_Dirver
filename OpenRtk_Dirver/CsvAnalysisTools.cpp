#include "CsvAnalysisTools.h"
#include <QDir>
#include <QUrl>
#include <QDesktopServices>
#include <QFileDialog>

CsvAnalysisTools::CsvAnalysisTools(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	setAcceptDrops(true);
	m_CsvAnalysisThread = new CsvAnalysisThread(this);
	connect(ui.select_btn, SIGNAL(clicked()), this, SLOT(onSelectFileClicked()));
	connect(ui.analysis_btn, SIGNAL(clicked()), this, SLOT(onAnalysisClicked()));

	connect(m_CsvAnalysisThread, SIGNAL(sgnProgress(int, int)), this, SLOT(onProcess(int, int)));
	connect(m_CsvAnalysisThread, SIGNAL(sgnFinished()), this, SLOT(onFinished()));
}

CsvAnalysisTools::~CsvAnalysisTools()
{
}

void CsvAnalysisTools::setOperable(bool enable)
{
	ui.filepath_edt->setEnabled(enable);
	ui.select_btn->setEnabled(enable);
	ui.analysis_btn->setEnabled(enable);
}

void CsvAnalysisTools::dragEnterEvent(QDragEnterEvent * event)
{
	event->acceptProposedAction();
}

void CsvAnalysisTools::dropEvent(QDropEvent * event)
{
	if (ui.filepath_edt->isEnabled()) {
		QString name = event->mimeData()->urls().first().toLocalFile();
		ui.filepath_edt->setText(name);
	}
}

void CsvAnalysisTools::onSelectFileClicked()
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

void CsvAnalysisTools::onAnalysisClicked()
{
	QString filename = ui.filepath_edt->text();
	if (filename.isEmpty()) {
		return;
	}
	ui.progressBar->setValue(0);
	m_CsvAnalysisThread->setFileFormat(ui.fileformat_cmb->currentIndex());
	m_CsvAnalysisThread->setFileName(filename);
	m_CsvAnalysisThread->start();
	setOperable(false);
}

void CsvAnalysisTools::onProcess(int present, int msecs)
{
	ui.progressBar->setValue(present);
	double dProgress = ui.progressBar->value() * 100.0 / ui.progressBar->maximum();
	ui.progressBar->setFormat(QString("%1%").arg(QString::number(dProgress, 'f', 2)));
	m_TimeShow.setHMS(0, 0, 0, 0);
	ui.time_lb->setText(m_TimeShow.addMSecs(msecs).toString("hh:mm:ss:zzz"));
}

void CsvAnalysisTools::onFinished()
{
	setOperable(true);
}

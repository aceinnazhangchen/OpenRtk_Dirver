#include "SplitTools.h"

SplitTools::SplitTools(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

SplitTools::~SplitTools()
{
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

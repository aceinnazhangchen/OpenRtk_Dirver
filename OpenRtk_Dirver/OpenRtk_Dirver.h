#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_OpenRtk_Dirver.h"
#include "StreamConfigUI.h"
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QFile>
#include <QTime>
#include "DecodeTools.h"
#include "MergeTools.h"
#include "AnalysisTools.h"

enum emStates
{
	emStart,
	emStop,
};

class OpenRtk_Dirver : public QMainWindow
{
    Q_OBJECT

public:
    OpenRtk_Dirver(QWidget *parent = Q_NULLPTR);
	~OpenRtk_Dirver();
	void getConfigFromUI();
	void changePortName();
protected:
	void dragEnterEvent(QDragEnterEvent * event);
	void dropEvent(QDropEvent * event);
private:
    Ui::OpenRtk_DirverClass ui;
	emStates m_nStates;
	QTime m_TimeShow;
	DecodeTools* m_DecodeToolsWidget;
	MergeTools* m_MergeTools;
	AnalysisTools* m_AnalysisTools;
public slots:
	void onSearchComPort();
	void onOpenClose();
	void onClear();
	void onShowReadWriteSize(int index, int read, int write);
	void onOpenLogPathClicked();
	void onSelectFileClicked();
	void onReplayProcess(int nProcess, int mSecs);
	void onDecodeWidgetOpen();
	void onMergeWidgetOpen();
	void onAnalysisWidgetOpen();
};

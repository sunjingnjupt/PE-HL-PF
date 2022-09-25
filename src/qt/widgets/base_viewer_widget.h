// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#pragma once

#include <QMainWindow>
#include "viewer.h"

class BaseViewerWidget : public QMainWindow {
  Q_OBJECT

public:
	explicit BaseViewerWidget(QWidget *parent);
	~BaseViewerWidget() override {}

protected:
	bool eventFilter(QObject *obj, QEvent *event) override;
	// virtual void mouseDoubleClickEvent(QMouseEvent *e);
	Viewer *_viewer = nullptr;

// public:
// 	bool isFullScreen;

// signals:
//     void fullScreen(bool);
};

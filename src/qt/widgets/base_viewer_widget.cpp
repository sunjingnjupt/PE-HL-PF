// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include "base_viewer_widget.h"

#include <QKeyEvent>

BaseViewerWidget::BaseViewerWidget(QWidget *parent) :
         QMainWindow(parent){}

bool BaseViewerWidget::eventFilter(QObject *object, QEvent *event) 
{
	// object = nullptr;
	if (event->type() == QEvent::KeyPress) 
	{
		QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
		if (keyEvent->key() == Qt::Key_Right || 
				keyEvent->key() == Qt::Key_Left ||
				keyEvent->key() == Qt::Key_1 ||
				keyEvent->key() == Qt::Key_2 ||
				keyEvent->key() == Qt::Key_3 ||
				keyEvent->key() == Qt::Key_Space) 
		{
			keyPressEvent(keyEvent);
			return true;
		} 
		else 
		{
			return false;
		}
	}
	return false;
}

// void BaseViewerWidget::mouseDoubleClickEvent(QMouseEvent *e)
// {
//     // 向父窗口发送是否全屏的信号
//     if (isFullScreen)
//     {
//         isFullScreen = false;
//         emit fullScreen(isFullScreen);
//     }
//     else
//     {
//         isFullScreen = true;
//         emit fullScreen(isFullScreen);
//     }
    
//     fprintf(stderr, "double clicked ---------\n\n");
// }
/*!
 This QWindowClass represents the rightmost screen window.
 It has an associated UI file.

 @author jdavis
 @date 02/21/2013

 TODO add layout
 */

#ifndef RIGHTWINDOW_H
#define RIGHTWINDOW_H

#include <QWidget>
#include <QKeyEvent>

#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>

#include <osgDB/ReadFile>

namespace Ui {
class RightWindow;
}

class RightWindow : public QWidget
{
    Q_OBJECT

public:
    explicit RightWindow();
    ~RightWindow();

private:
    void keyPressEvent(QKeyEvent *key);

};

#endif // RIGHTWINDOW_H

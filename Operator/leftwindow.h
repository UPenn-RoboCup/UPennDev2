/*!
 This QWindowClass represents the leftmost screen window.
 It has an associated UI file.

 @author jdavis
 @date 02/21/2013

 TODO add layout
 */

#ifndef LEFTWINDOW_H
#define LEFTWINDOW_H

#include <QWidget>
#include <QKeyEvent>

#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>

#include <osgDB/ReadFile>

namespace Ui {
class LeftWindow;
}

class LeftWindow : public QWidget
{
    Q_OBJECT

public:
    explicit LeftWindow();
    ~LeftWindow();

    void keyPressEvent(QKeyEvent *key);

};
#endif // LEFTWINDOW_H

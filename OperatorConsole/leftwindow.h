/*!
 This QWindowClass represents the leftmost screen window.
 It has an associated UI file.

 @author jdavis
 @date 02/21/2013

 TODO add layout
 */

#ifndef LEFTWINDOW_H
#define LEFTWINDOW_H

#include <QMainWindow>

namespace Ui {
class LeftWindow;
}

class LeftWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit LeftWindow(QWidget *parent = 0);
    ~LeftWindow();
    
private:
    Ui::LeftWindow *ui;
};

#endif // LEFTWINDOW_H

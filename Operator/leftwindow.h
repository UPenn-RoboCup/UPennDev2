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

namespace Ui {
class LeftWindow;
}

class LeftWindow : public QWidget
{
    Q_OBJECT

public:
    explicit LeftWindow();
    ~LeftWindow();
    void initialize();
};
#endif // LEFTWINDOW_H

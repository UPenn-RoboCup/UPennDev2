/*!
 This QWidget represents and engineering control widget.
 It encapsulates control and status indicators for
 off nominal operation.

 initial class
 @author jdavis
 @date 02/21/2013
 */

#ifndef ENGINEERINGCONTROLWIDGET_H
#define ENGINEERINGCONTROLWIDGET_H

#include <QWidget>

namespace Ui {
class EngineeringControlWidget;
}

class EngineeringControlWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit EngineeringControlWidget(QWidget *parent = 0);
    ~EngineeringControlWidget();
    
private:
    Ui::EngineeringControlWidget *ui;
};

#endif // ENGINEERINGCONTROLWIDGET_H

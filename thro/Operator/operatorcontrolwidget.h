/*!
 This QWidget class represents the operator controls.
 It is diplayed on the touchscreen monitor.

 It has an associated UI file.

 @author jdavis
 @date 02/21/2013

 TODO add controls, state machine model, layout
 */

#ifndef OPERATORCONTROLWIDGET_H
#define OPERATORCONTROLWIDGET_H

#include <QWidget>

namespace Ui {
class OperatorControlWidget;
}

class OperatorControlWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit OperatorControlWidget(QWidget *parent = 0);
    ~OperatorControlWidget();
    
private:
    Ui::OperatorControlWidget *ui;
};

#endif // OPERATORCONTROLWIDGET_H

#include "operatorcontrolwidget.h"
#include "ui_operatorcontrolwidget.h"

OperatorControlWidget::OperatorControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::OperatorControlWidget)
{
    ui->setupUi(this);
}

OperatorControlWidget::~OperatorControlWidget()
{
    delete ui;
}

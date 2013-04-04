#include "engineeringcontrolwidget.h"
#include "ui_engineeringcontrolwidget.h"

EngineeringControlWidget::EngineeringControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EngineeringControlWidget)
{
    ui->setupUi(this);
}

EngineeringControlWidget::~EngineeringControlWidget()
{
    delete ui;
}

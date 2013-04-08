#-------------------------------------------------
#
# Project created by QtCreator 2013-02-22T13:30:46
#
#-------------------------------------------------

QT += core gui
QT += phonon
QT += opengl
QT += osg
QT += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OperatorConsole
TEMPLATE = app


SOURCES += main.cpp\
        centerwindow.cpp \
    rightwindow.cpp \
    leftwindow.cpp \
    datamanager.cpp \
    osgwidget.cpp \
    osgcloudwidget.cpp \
    osgmodelwidget.cpp \
    videowidget.cpp \
    operatorcontrolwidget.cpp \
    engineeringcontrolwidget.cpp \
    visualdatabase.cpp \
    videomanager.cpp \
    eventmanager.cpp \
    behaviorstatemachine.cpp \
    ingressstatemachine.cpp \
    egressstatemachine.cpp \
    climbstatemachine.cpp \
    walkstatemachine.cpp \
    osgcamera.cpp \
    commsinterface.cpp \
    osgrenderthread.cpp \
    osgmousemanipulator.cpp \
    manager.cpp \
    keyboardeventfilter.cpp \
    modelmanipulator.cpp \
    videofeed.cpp \
    luaengine.cpp

HEADERS  += centerwindow.h \
    rightwindow.h \
    leftwindow.h \
    datamanager.h \
    osgwidget.h \
    osgcloudwidget.h \
    osgmodelwidget.h \
    videowidget.h \
    operatorcontrolwidget.h \
    engineeringcontrolwidget.h \
    visualdatabase.h \
    videomanager.h \
    eventmanager.h \
    behaviorstatemachine.h \
    ingressstatemachine.h \
    egressstatemachine.h \
    climbstatemachine.h \
    walkstatemachine.h \
    osgcamera.h \
    commsinterface.h \
    osgrenderthread.h \
    osgmousemanipulator.h\
    manager.h \
    keyboardeventfilter.h \
    modelmanipulator.h \
    videofeed.h \
    luaengine.h

FORMS    += centerwindow.ui \
    rightwindow.ui \
    leftwindow.ui \
    operatorcontrolwidget.ui \
    engineeringcontrolwidget.ui

OTHER_FILES += \
    doxyfile.config \
    INSTALL.txt

LIBS += -lqwt -lOpenThreads -losg -losgDB -losgUtil -losgViewer -losgGA -losgQt -llualib

INCLUDEPATH +=  /usr/include/lua5.1
